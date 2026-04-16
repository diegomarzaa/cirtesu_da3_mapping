#!/usr/bin/env python3
"""
incremental_mapper_node — Streams camera frames, processes chunks with DA3
while new frames keep coming, and publishes the growing pointcloud in RViz.

Lifecycle
---------
    IDLE ──start──► RUNNING ──stop──► IDLE
                       │
                       ├─ image sub: saves PNGs into the session dir
                       └─ worker thread: assembles chunks, calls DA3Engine,
                                         appends result to the accumulated
                                         pointcloud and republishes it.

Services
--------
    ~/start (std_srvs/Trigger) — begin a new session (model stays loaded).
    ~/stop  (std_srvs/Trigger) — stop capture, drain in-flight work.

Topics in
---------
    <image_topic>  sensor_msgs/CompressedImage  (default /image_raw/compressed)

Topics out
----------
    /cirtesu/map_pointcloud   sensor_msgs/PointCloud2       (accumulated)
    /cirtesu/camera_path      nav_msgs/Path                 (accumulated)
    ~/status                  std_msgs/String               (human-readable)

Notes
-----
    Loop closure is disabled — each chunk is aligned only with the previous
    one and then composed through the cumulative sim3 into the chunk-0 frame.
    Accumulated drift is expected on long trajectories; good enough for
    incremental visualization.
"""
from __future__ import annotations

import os
import sys
import threading
from datetime import datetime
from enum import Enum, auto
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

# Make cirtesu_da3_mapping importable regardless of how the script is launched.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_PARENT = os.path.abspath(os.path.join(_SCRIPT_DIR, os.pardir))
if _PKG_PARENT not in sys.path:
    sys.path.insert(0, _PKG_PARENT)

from cirtesu_da3_mapping.da3_engine import ChunkResult, DA3Engine  # noqa: E402

_IMAGE_TOPIC = "/image_raw/compressed"
_POINTCLOUD_TOPIC = "/cirtesu/map_pointcloud"
_PATH_TOPIC = "/cirtesu/camera_path"
_FRAME_ID = "da3_world"
_TARGET_SAVE_FPS = 1.0
_DEBUG_SAVE = False
_VOXEL_SIZE = 0.01


# ──────────────────────────────────────────────────────────────────────────────
# Terminal colors
# ──────────────────────────────────────────────────────────────────────────────

_RESET, _BOLD = '\033[0m', '\033[1m'
_RED, _GREEN, _YELLOW = '\033[31m', '\033[32m', '\033[33m'
_BLUE, _MAGENTA, _CYAN = '\033[34m', '\033[35m', '\033[36m'


def color(text: str, c: str, bold: bool = False) -> str:
    return f'{_BOLD}{c}{text}{_RESET}' if bold else f'{c}{text}{_RESET}'


# ──────────────────────────────────────────────────────────────────────────────
# State
# ──────────────────────────────────────────────────────────────────────────────

class State(Enum):
    IDLE = auto()
    RUNNING = auto()
    ERROR = auto()


# ──────────────────────────────────────────────────────────────────────────────
# PointCloud2 packing
# ──────────────────────────────────────────────────────────────────────────────

_PC_FIELDS = [
    PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.UINT32,  count=1),
]


def build_pointcloud2(points: np.ndarray, colors: np.ndarray, header: Header) -> PointCloud2:
    r = colors[:, 0].astype(np.uint32)
    g = colors[:, 1].astype(np.uint32)
    b = colors[:, 2].astype(np.uint32)
    structured = np.empty(len(points), dtype=point_cloud2.dtype_from_fields(_PC_FIELDS))
    structured['x'] = points[:, 0]
    structured['y'] = points[:, 1]
    structured['z'] = points[:, 2]
    structured['rgb'] = (r << 16) | (g << 8) | b
    return point_cloud2.create_cloud(header, _PC_FIELDS, structured)


# ──────────────────────────────────────────────────────────────────────────────
# Rotation → quaternion (same formula as in frame_recorder_node)
# ──────────────────────────────────────────────────────────────────────────────

def rot_to_quat(R: np.ndarray):
    tr = np.trace(R)
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        return (R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s, \
               (R[1, 0] - R[0, 1]) * s, 0.25 / s
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(max(1.0 + R[0, 0] - R[1, 1] - R[2, 2], 1e-12))
        return 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, \
               (R[2, 1] - R[1, 2]) / s
    if R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(max(1.0 + R[1, 1] - R[0, 0] - R[2, 2], 1e-12))
        return (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, \
               (R[0, 2] - R[2, 0]) / s
    s = 2.0 * np.sqrt(max(1.0 + R[2, 2] - R[0, 0] - R[1, 1], 1e-12))
    return (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, \
           (R[1, 0] - R[0, 1]) / s


# ──────────────────────────────────────────────────────────────────────────────
# Voxel downsampling — keeps one point per voxel cell (no extra deps)
# ──────────────────────────────────────────────────────────────────────────────

def _voxel_downsample(pts: np.ndarray, clrs: np.ndarray, voxel_size: float):
    """Keep one point per voxel. Fast numpy implementation, no Open3D needed.
    Voxel size 0.02, from 7.9M to ~500K-1M points...
    Voxel size 0.05 if still heavy, 0.1 for more 
    """
    keys = np.floor(pts / voxel_size).astype(np.int64)
    # Encode (i,j,k) as a single int64 for np.unique
    mn = keys.min(axis=0)
    keys -= mn
    mx = keys.max(axis=0) + 1
    flat = keys[:, 0] * (mx[1] * mx[2]) + keys[:, 1] * mx[2] + keys[:, 2]
    _, idx = np.unique(flat, return_index=True)
    return pts[idx], clrs[idx]


# ──────────────────────────────────────────────────────────────────────────────
# Simple PLY writer (only used when debug_save is on)
# ──────────────────────────────────────────────────────────────────────────────

def save_ply_xyzrgb(path: str, points: np.ndarray, colors: np.ndarray) -> None:
    with open(path, 'wb') as f:
        header = (
            f'ply\nformat binary_little_endian 1.0\n'
            f'element vertex {len(points)}\n'
            f'property float x\nproperty float y\nproperty float z\n'
            f'property uchar red\nproperty uchar green\nproperty uchar blue\n'
            f'end_header\n'
        ).encode()
        f.write(header)
        rows = np.empty(len(points), dtype=[
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
            ('r', 'u1'), ('g', 'u1'), ('b', 'u1'),
        ])
        rows['x'], rows['y'], rows['z'] = points[:, 0], points[:, 1], points[:, 2]
        rows['r'], rows['g'], rows['b'] = colors[:, 0], colors[:, 1], colors[:, 2]
        f.write(rows.tobytes())


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────

class IncrementalMapperNode(Node):
    def __init__(self):
        super().__init__('incremental_mapper')

        self.declare_parameter('image_topic', _IMAGE_TOPIC)
        self.declare_parameter('session_base_dir', str(Path.home() / 'da3_sessions'))
        self.declare_parameter('da3_streaming_dir', '')
        self.declare_parameter('da3_src_dir', '')
        self.declare_parameter('da3_config', '')
        self.declare_parameter('pointcloud_topic', _POINTCLOUD_TOPIC)
        self.declare_parameter('path_topic', _PATH_TOPIC)
        self.declare_parameter('frame_id', _FRAME_ID)
        self.declare_parameter('target_save_fps', _TARGET_SAVE_FPS)
        self.declare_parameter('debug_save', _DEBUG_SAVE)
        self.declare_parameter('voxel_downsample', _VOXEL_SIZE)

        self._image_topic = self.get_parameter('image_topic').value
        self._session_base = self.get_parameter('session_base_dir').value
        self._da3_streaming_dir = self.get_parameter('da3_streaming_dir').value
        self._da3_src_dir = self.get_parameter('da3_src_dir').value
        self._da3_config = self.get_parameter('da3_config').value
        self._pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self._path_topic = self.get_parameter('path_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._target_save_fps = float(self.get_parameter('target_save_fps').value)
        self._debug_save = bool(self.get_parameter('debug_save').value)
        self._voxel_size = float(self.get_parameter('voxel_downsample').value)

        # Latched QoS: late subscribers still get the last sample.
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._pc_pub = self.create_publisher(PointCloud2, self._pointcloud_topic, latched)
        self._path_pub = self.create_publisher(NavPath, self._path_topic, latched)
        self._status_pub = self.create_publisher(String, '~/status', 10)

        self.create_timer(1.0, self._publish_status)
        self.create_service(Trigger, '~/start', self._srv_start)
        self.create_service(Trigger, '~/stop', self._srv_stop)

        # ── State ──────────────────────────────────────────────────────────────
        self._state = State.IDLE
        self._state_lock = threading.Lock()

        # Session-scoped
        self._session_dir: str | None = None
        self._frames_dir: str | None = None
        self._debug_dir: str | None = None

        # Frame buffer (produced by image callback, consumed by worker)
        self._frame_paths: list[str] = []
        self._frames_cv = threading.Condition()
        self._last_saved_ns = 0
        self._min_period_ns = (
            int(1e9 / self._target_save_fps) if self._target_save_fps > 0 else 0
        )

        # Accumulated map (produced by worker, read by publisher)
        self._map_chunks: list[tuple[np.ndarray, np.ndarray]] = []  # (points, colors)
        self._map_poses: list[np.ndarray] = []  # list of (4, 4) c2w matrices
        self._map_lock = threading.Lock()

        # Worker
        self._worker: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._img_sub = None

        # Engine (loaded once; reused across sessions)
        self.get_logger().info(color('Loading DA3 model...', _CYAN, bold=True))
        self._engine = DA3Engine(
            config_path=self._da3_config,
            da3_streaming_dir=self._da3_streaming_dir,
            da3_src_dir=self._da3_src_dir,
        )
        self.get_logger().info(color(
            f'DA3 model ready (chunk_size={self._engine.chunk_size}, '
            f'overlap={self._engine.overlap}, device={self._engine.device}).',
            _CYAN, bold=True,
        ))
        self.get_logger().info(
            f'Input topic: {self._image_topic} | session_base_dir: {self._session_base}'
        )
        self.get_logger().info(
            '→ ros2 service call /incremental_mapper/start std_srvs/srv/Trigger {}'
        )

    # ── Services ───────────────────────────────────────────────────────────────

    def _srv_start(self, _req, response):
        with self._state_lock:
            if self._state == State.RUNNING:
                return self._fail(response, 'Already running.')
            self._prepare_session()
            self._state = State.RUNNING

        self._stop_event.clear()
        self._img_sub = self.create_subscription(
            CompressedImage, self._image_topic, self._image_callback, 10
        )
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(color(
            f'RUNNING — session {self._session_dir}', _GREEN, bold=True,
        ))
        response.success = True
        response.message = f'Running. Session: {self._session_dir}'
        return response

    def _srv_stop(self, _req, response):
        with self._state_lock:
            if self._state != State.RUNNING:
                return self._fail(response, f'Not running (state={self._state.name}).')

        if self._img_sub is not None:
            self.destroy_subscription(self._img_sub)
            self._img_sub = None

        self._stop_event.set()
        with self._frames_cv:
            self._frames_cv.notify_all()

        # Wait for the worker to finish the in-flight chunk (can take a few
        # seconds). We do not use a timeout: otherwise a subsequent start()
        # could race against a still-running worker.
        if self._worker is not None:
            self._worker.join()
            self._worker = None

        with self._state_lock:
            self._state = State.IDLE

        self.get_logger().info(color('STOPPED', _YELLOW, bold=True))
        response.success = True
        response.message = 'Stopped.'
        return response

    # ── Session setup ──────────────────────────────────────────────────────────

    def _prepare_session(self):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._session_dir = os.path.join(self._session_base, f'session_{ts}')
        self._frames_dir = os.path.join(self._session_dir, 'frames')
        os.makedirs(self._frames_dir, exist_ok=True)
        if self._debug_save:
            self._debug_dir = os.path.join(self._session_dir, 'debug')
            os.makedirs(self._debug_dir, exist_ok=True)
        else:
            self._debug_dir = None

        self._frame_paths = []
        self._last_saved_ns = 0
        with self._map_lock:
            self._map_chunks.clear()
            self._map_poses.clear()
        self._engine.reset()

    # ── Image capture ──────────────────────────────────────────────────────────

    def _image_callback(self, msg: CompressedImage):
        with self._state_lock:
            if self._state != State.RUNNING:
                return

        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds

        if self._min_period_ns > 0 and self._last_saved_ns > 0 \
                and (stamp_ns - self._last_saved_ns) < self._min_period_ns:
            return
        self._last_saved_ns = stamp_ns

        try:
            path = self._save_image(msg, len(self._frame_paths))
        except Exception as e:
            self.get_logger().warning(f'{color("SAVE FAILED", _RED, bold=True)}: {e}')
            return

        with self._frames_cv:
            self._frame_paths.append(path)
            n = len(self._frame_paths)
            self._frames_cv.notify_all()

        self.get_logger().info(
            f'{color("frame", _GREEN)} #{n - 1:06d} saved ({os.path.basename(path)})'
        )

    def _save_image(self, msg: CompressedImage, idx: int) -> str:
        import cv2
        path = os.path.join(self._frames_dir, f'{idx:06d}.png')
        # Decompress CompressedImage directly with cv2
        cv_img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        cv2.imwrite(path, cv_img)
        return path

    # ── Worker loop ────────────────────────────────────────────────────────────

    def _worker_loop(self):
        """Consumer: assembles chunks from the frame buffer and processes them."""
        chunk_size = self._engine.chunk_size
        overlap = self._engine.overlap
        next_start = 0  # index of the first frame of the current chunk

        while not self._stop_event.is_set():
            # Wait until the buffer has a full chunk or stop is requested.
            with self._frames_cv:
                while (
                    not self._stop_event.is_set()
                    and len(self._frame_paths) < next_start + chunk_size
                ):
                    self._frames_cv.wait(timeout=0.25)
                if self._stop_event.is_set():
                    break
                chunk_paths = list(self._frame_paths[next_start:next_start + chunk_size])

            try:
                result = self._engine.process_chunk(chunk_paths)
            except Exception as e:
                self.get_logger().error(color(
                    f'Chunk {next_start}..{next_start + chunk_size} failed: {e}',
                    _RED, bold=True,
                ))
                with self._state_lock:
                    self._state = State.ERROR
                return

            self._integrate_and_publish(result)
            if self._debug_save:
                self._debug_dump(result)

            next_start += chunk_size - overlap

        self.get_logger().info(color('Worker exited.', _BLUE))

    # ── Map accumulation and publishing ────────────────────────────────────────

    def _integrate_and_publish(self, result: ChunkResult) -> None:
        with self._map_lock:
            if len(result.points) > 0:
                self._map_chunks.append((result.points, result.colors))
            for pose in result.poses_c2w:
                self._map_poses.append(pose)
            total_points = sum(len(p) for p, _ in self._map_chunks)
            total_poses = len(self._map_poses)

        stamp = self.get_clock().now().to_msg()
        header = Header(frame_id=self._frame_id, stamp=stamp)

        self._publish_accumulated_pointcloud(header)
        self._publish_accumulated_path(header)

        self.get_logger().info(color(
            f'[chunk {result.chunk_idx}] published — '
            f'{total_points} pts, {total_poses} poses',
            _MAGENTA, bold=True,
        ))

    def _publish_accumulated_pointcloud(self, header: Header) -> None:
        with self._map_lock:
            if not self._map_chunks:
                return
            pts = np.concatenate([p for p, _ in self._map_chunks], axis=0)
            clr = np.concatenate([c for _, c in self._map_chunks], axis=0)
        if self._voxel_size > 0.0:
            pts, clr = _voxel_downsample(pts, clr, self._voxel_size)
        self._pc_pub.publish(build_pointcloud2(pts, clr, header))

    def _publish_accumulated_path(self, header: Header) -> None:
        with self._map_lock:
            poses = list(self._map_poses)
        if not poses:
            return
        path_msg = NavPath(header=header)
        for c2w in poses:
            ps = PoseStamped(header=header)
            ps.pose.position.x = float(c2w[0, 3])
            ps.pose.position.y = float(c2w[1, 3])
            ps.pose.position.z = float(c2w[2, 3])
            qx, qy, qz, qw = rot_to_quat(c2w[:3, :3])
            ps.pose.orientation.x = float(qx)
            ps.pose.orientation.y = float(qy)
            ps.pose.orientation.z = float(qz)
            ps.pose.orientation.w = float(qw)
            path_msg.poses.append(ps)
        self._path_pub.publish(path_msg)

    # ── Debug dump ─────────────────────────────────────────────────────────────

    def _debug_dump(self, result: ChunkResult) -> None:
        if self._debug_dir is None:
            return
        ply_path = os.path.join(self._debug_dir, f'chunk_{result.chunk_idx:04d}.ply')
        save_ply_xyzrgb(ply_path, result.points, result.colors)

        meta_path = os.path.join(self._debug_dir, f'chunk_{result.chunk_idx:04d}.npz')
        s, R, t = result.sim3_cum
        np.savez_compressed(
            meta_path,
            sim3_s=np.asarray(s, dtype=np.float64),
            sim3_R=np.asarray(R, dtype=np.float64),
            sim3_t=np.asarray(t, dtype=np.float64),
            poses_c2w=result.poses_c2w,
            num_points=len(result.points),
        )

    # ── Status ─────────────────────────────────────────────────────────────────

    def _publish_status(self):
        with self._state_lock:
            state = self._state
        with self._map_lock:
            n_chunks = len(self._map_chunks)
            n_points = sum(len(p) for p, _ in self._map_chunks)
            n_poses = len(self._map_poses)
        n_frames = len(self._frame_paths)
        msg = {
            State.IDLE: 'IDLE — call /incremental_mapper/start to begin',
            State.RUNNING: (
                f'RUNNING — frames={n_frames}, chunks={n_chunks}, '
                f'points={n_points}, poses={n_poses}'
            ),
            State.ERROR: 'ERROR — worker failed, see log',
        }[state]
        self._status_pub.publish(String(data=msg))

    # ── Helpers ────────────────────────────────────────────────────────────────

    @staticmethod
    def _fail(response, msg: str):
        response.success = False
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = IncrementalMapperNode()
    except Exception as e:
        print(f'[FATAL] {e}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
