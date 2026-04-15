#!/usr/bin/env python3
"""
frame_recorder_node — Records camera frames and processes them with DA3-Streaming.

State machine
-------------
  IDLE → RECORDING → PROCESSING → DONE
                                 → ERROR

Services
--------
  ~/start_recording  (std_srvs/Trigger):
      Creates a timestamped session directory and starts saving incoming frames.

  ~/stop_and_process (std_srvs/Trigger):
      Stops saving frames and spawns DA3-Streaming in a background thread.
      Returns immediately; the processing state is visible on ~/status.

Topics in
---------
  <image_topic>  sensor_msgs/Image  — camera frames (configurable, default /camera/image_raw)

Topics out
----------
  /cirtesu/map_pointcloud   sensor_msgs/PointCloud2       — published once processing is done
  /cirtesu/camera_path      nav_msgs/Path                 — camera trajectory
  /cirtesu/camera_axes      visualization_msgs/MarkerArray — RGB axes per camera pose
  /cirtesu/camera_frustums  visualization_msgs/MarkerArray — frustum per camera pose
  ~/status                  std_msgs/String               — human-readable state + frame count

Workflow
--------
  1. Launch: ros2 launch cirtesu_da3_mapping record_and_map.launch.py
  2. Start:  ros2 service call /frame_recorder/start_recording std_srvs/srv/Trigger {}
  3. Move the robot / camera.
  4. Stop:   ros2 service call /frame_recorder/stop_and_process std_srvs/srv/Trigger {}
  5. Watch:  ros2 topic echo /frame_recorder/status
  6. Result appears in RViz automatically when processing completes.
"""
import os
import subprocess
import sys
import threading
from datetime import datetime
from enum import Enum, auto

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

# Default paths inside the da3-ros2-wrapper container.
# On the host, /home/diego/Documents/02-Universidad/Cirtesu is mounted as /home/diego/Cirtesu.
_DA3_PYTHON = '/opt/venvs/da3/bin/python3'
_DA3_SCRIPT = (
    '/home/diego/Cirtesu/Repositories/Depth-Anything-3/'
    'da3_streaming/da3_streaming.py'
)
_DA3_CONFIG = (
    '/home/diego/Cirtesu/Repositories/Depth-Anything-3/'
    'da3_streaming/configs/diego_config.yaml'
)
_SESSION_BASE = '/home/diego/Cirtesu/media/da3_sessions'


# ──────────────────────────────────────────────────────────────────────────────
# ANSI colors for terminal logs
# ──────────────────────────────────────────────────────────────────────────────

_RESET = '\033[0m'
_BOLD = '\033[1m'
_RED = '\033[31m'
_GREEN = '\033[32m'
_YELLOW = '\033[33m'
_BLUE = '\033[34m'
_MAGENTA = '\033[35m'
_CYAN = '\033[36m'


def colorize(text: str, color: str, bold: bool = False) -> str:
    prefix = f'{_BOLD}{color}' if bold else color
    return f'{prefix}{text}{_RESET}'


# ──────────────────────────────────────────────────────────────────────────────
# State
# ──────────────────────────────────────────────────────────────────────────────

class State(Enum):
    IDLE = auto()
    RECORDING = auto()
    PROCESSING = auto()
    DONE = auto()
    ERROR = auto()


# ──────────────────────────────────────────────────────────────────────────────
# PLY / PointCloud2 helpers (same logic as ply_publisher_node)
# ──────────────────────────────────────────────────────────────────────────────

def load_ply(path: str, voxel_size: float):
    """Load a .ply file with Open3D and return (points[N,3] float32, colors[N,3] uint8)."""
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(path)
    if len(pcd.points) == 0:
        raise RuntimeError(f'PLY is empty or unreadable: {path}')
    if voxel_size > 0.0:
        pcd = pcd.voxel_down_sample(voxel_size)
    points = np.asarray(pcd.points, dtype=np.float32)
    if pcd.has_colors():
        colors = (np.asarray(pcd.colors) * 255.0).astype(np.uint8)
    else:
        colors = np.full((len(points), 3), 255, dtype=np.uint8)
    return points, colors


def build_pointcloud2(points, colors, header):
    """Pack XYZ + RGB into a sensor_msgs/PointCloud2 (16 bytes/point, 0x00RRGGBB packing)."""
    fields = [
        PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32,  count=1),
    ]
    r = colors[:, 0].astype(np.uint32)
    g = colors[:, 1].astype(np.uint32)
    b = colors[:, 2].astype(np.uint32)
    structured = np.empty(len(points), dtype=point_cloud2.dtype_from_fields(fields))
    structured['x'] = points[:, 0]
    structured['y'] = points[:, 1]
    structured['z'] = points[:, 2]
    structured['rgb'] = (r << 16) | (g << 8) | b
    return point_cloud2.create_cloud(header, fields, structured)


# ──────────────────────────────────────────────────────────────────────────────
# Camera pose helpers (same logic as camera_poses_publisher_node)
# ──────────────────────────────────────────────────────────────────────────────

def read_camera_poses(path):
    poses = []
    with open(path) as f:
        for line in f:
            vals = list(map(float, line.strip().split()))
            if len(vals) == 16:
                poses.append(np.array(vals, dtype=np.float64).reshape(4, 4))
    return poses


def read_intrinsics(path):
    intrinsics = []
    with open(path) as f:
        for line in f:
            vals = list(map(float, line.strip().split()))
            if len(vals) == 4:
                intrinsics.append(tuple(vals))  # (fx, fy, cx, cy)
    return intrinsics


def rot_to_quat(R):
    """Convert 3×3 rotation matrix to quaternion (x, y, z, w)."""
    tr = np.trace(R)
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        return (R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s, 0.25 / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(max(1.0 + R[0, 0] - R[1, 1] - R[2, 2], 1e-12))
        return 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(max(1.0 + R[1, 1] - R[0, 0] - R[2, 2], 1e-12))
        return (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s
    else:
        s = 2.0 * np.sqrt(max(1.0 + R[2, 2] - R[0, 0] - R[1, 1], 1e-12))
        return (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s


def make_point(xyz):
    p = Point()
    p.x, p.y, p.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
    return p


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────

class FrameRecorderNode(Node):
    def __init__(self):
        super().__init__('frame_recorder')

        self.declare_parameter('image_topic',         '/camera/image_raw')
        self.declare_parameter('session_base_dir',    _SESSION_BASE)
        self.declare_parameter('da3_python',          _DA3_PYTHON)
        self.declare_parameter('da3_script',          _DA3_SCRIPT)
        self.declare_parameter('da3_config',          _DA3_CONFIG)
        self.declare_parameter('target_save_fps', 1.0)
        self.declare_parameter('voxel_downsample',    0.0)
        self.declare_parameter('pointcloud_topic',    '/cirtesu/map_pointcloud')
        self.declare_parameter('frame_id',            'da3_world')
        self.declare_parameter('axis_length',         0.06)
        self.declare_parameter('frustum_depth',       0.18)

        self._image_topic   = self.get_parameter('image_topic').value
        self._session_base  = self.get_parameter('session_base_dir').value
        self._da3_python    = self.get_parameter('da3_python').value
        self._da3_script    = self.get_parameter('da3_script').value
        self._da3_config    = self.get_parameter('da3_config').value
        self._target_save_fps = float(self.get_parameter('target_save_fps').value)
        self._voxel         = float(self.get_parameter('voxel_downsample').value)
        self._pc_topic      = self.get_parameter('pointcloud_topic').value
        self._frame_id      = self.get_parameter('frame_id').value
        self._axis_len      = float(self.get_parameter('axis_length').value)
        self._frustum_depth = float(self.get_parameter('frustum_depth').value)

        # ── State ──────────────────────────────────────────────────────────────
        self._state      = State.IDLE
        self._state_lock = threading.Lock()
        self._frame_count  = 0   # frames received since start_recording
        self._total_saved  = 0   # frames actually written to disk
        self._last_saved_stamp_ns = None
        self._session_dir: str | None = None
        self._frames_dir:  str | None = None
        self._img_sub = None  # created/destroyed on demand
        self._min_save_period_ns = 0
        if self._target_save_fps > 0.0:
            self._min_save_period_ns = int(1e9 / self._target_save_fps)

        # ── QoS for latched result topics ─────────────────────────────────────
        # TRANSIENT_LOCAL so RViz receives the last sample even if it connects
        # after the publisher has already sent the one-shot message.
        self._result_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # ── Result publishers ─────────────────────────────────────────────────
        self._pc_pub       = self.create_publisher(PointCloud2,  self._pc_topic,              self._result_qos)
        self._path_pub     = self.create_publisher(NavPath,      '/cirtesu/camera_path',      self._result_qos)
        self._axes_pub     = self.create_publisher(MarkerArray,  '/cirtesu/camera_axes',      self._result_qos)
        self._frustums_pub = self.create_publisher(MarkerArray,  '/cirtesu/camera_frustums',  self._result_qos)

        # ── Status topic ──────────────────────────────────────────────────────
        self._status_pub   = self.create_publisher(String, '~/status', 10)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        # ── Services ──────────────────────────────────────────────────────────
        self.create_service(Trigger, '~/start_recording',  self._srv_start_recording)
        self.create_service(Trigger, '~/stop_and_process', self._srv_stop_and_process)

        self.get_logger().info(
            f'{colorize("frame_recorder ready", _CYAN, bold=True)} '
            f'(image_topic={self._image_topic}).\n'
            f'  → ros2 service call /{self.get_name()}/start_recording std_srvs/srv/Trigger {{}}')

    # ── Service: start_recording ───────────────────────────────────────────────

    def _srv_start_recording(self, _req, response):
        with self._state_lock:
            if self._state == State.RECORDING:
                return self._fail(response, 'Already recording.')
            if self._state not in (State.IDLE, State.DONE, State.ERROR):
                return self._fail(response, f'Cannot start while in state {self._state.name}.')

            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._session_dir = os.path.join(self._session_base, f'session_{ts}')
            self._frames_dir  = os.path.join(self._session_dir, 'frames')
            os.makedirs(self._frames_dir, exist_ok=True)

            self._frame_count = 0
            self._total_saved = 0
            self._last_saved_stamp_ns = None
            self._state = State.RECORDING

        self._img_sub = self.create_subscription(
            Image, self._image_topic, self._image_callback, 10
        )
        self.get_logger().info(
            f'{colorize("RECORDING started", _GREEN, bold=True)} → {self._session_dir}'
        )
        response.success = True
        response.message = f'Recording. Session: {self._session_dir}'
        return response

    # ── Service: stop_and_process ──────────────────────────────────────────────

    def _srv_stop_and_process(self, _req, response):
        with self._state_lock:
            if self._state != State.RECORDING:
                return self._fail(response, f'Not recording (state={self._state.name}).')
            if self._total_saved < 1:
                return self._fail(response, 'No frames saved yet — move the camera first.')
            saved = self._total_saved
            self._state = State.PROCESSING

        if self._img_sub is not None:
            self.destroy_subscription(self._img_sub)
            self._img_sub = None

        self.get_logger().info(
            f'{colorize("RECORDING stopped", _YELLOW, bold=True)}. '
            f'{saved} frames saved. {colorize("Launching DA3-Streaming...", _MAGENTA, bold=True)}'
        )
        threading.Thread(target=self._run_da3_streaming, daemon=True).start()

        response.success = True
        response.message = (
            f'Processing {saved} frames with DA3-Streaming. '
            f'Watch /{self.get_name()}/status for progress.'
        )
        return response

    # ── Image callback ─────────────────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        with self._state_lock:
            if self._state != State.RECORDING:
                return
            self._frame_count += 1

            stamp_ns = self._message_stamp_ns(msg)
            if (
                self._min_save_period_ns > 0
                and self._last_saved_stamp_ns is not None
                and (stamp_ns - self._last_saved_stamp_ns) < self._min_save_period_ns
            ):
                return
            idx = self._total_saved
            self._total_saved += 1
            self._last_saved_stamp_ns = stamp_ns

        try:
            saved_path = self._save_image(msg, idx)
            self.get_logger().info(
                f'{colorize("SAVED", _GREEN, bold=True)} frame {idx:06d} '
                f'(received={self._frame_count}, saved={self._total_saved}) -> {saved_path}'
            )
        except Exception as e:
            self.get_logger().warning(
                f'{colorize("SAVE FAILED", _RED, bold=True)} frame {idx:06d}: {e}'
            )

    def _message_stamp_ns(self, msg: Image) -> int:
        """Prefer the image timestamp; fall back to the node clock if missing/invalid."""
        stamp = msg.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns > 0:
            return stamp_ns
        return self.get_clock().now().nanoseconds

    def _save_image(self, msg: Image, idx: int):
        """Convert sensor_msgs/Image → PNG and write to the frames directory."""
        path = os.path.join(self._frames_dir, f'{idx:06d}.png')

        # Preferred path: cv_bridge (standard in ROS2 desktop)
        try:
            from cv_bridge import CvBridge
            import cv2
            cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(path, cv_img)
            return path
        except Exception:
            pass

        # Fallback: manual conversion for the most common encodings
        enc  = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        h, w = msg.height, msg.width

        if enc in ('rgb8', 'bgr8'):
            img = data.reshape(h, w, 3)
            if enc == 'rgb8':
                img = img[:, :, ::-1]           # RGB → BGR for OpenCV/PIL
        elif enc == 'mono8':
            img = data.reshape(h, w)
        elif enc in ('rgba8', 'bgra8'):
            img = data.reshape(h, w, 4)[:, :, :3]
            if enc == 'rgba8':
                img = img[:, :, ::-1]
        else:
            raise ValueError(f'Unsupported image encoding: {enc}')

        try:
            import cv2
            cv2.imwrite(path, img)
        except ImportError:
            from PIL import Image as PILImage
            # PIL needs RGB; img is currently BGR
            if enc != 'mono8':
                img = img[:, :, ::-1]
            mode = 'L' if enc == 'mono8' else 'RGB'
            PILImage.fromarray(img, mode).save(path)

        return path

    # ── DA3-Streaming subprocess ───────────────────────────────────────────────

    def _run_da3_streaming(self):
        """
        Run da3_streaming.py in a subprocess, then publish results.

        Called from a daemon thread so it does not block the ROS2 spin loop.
        The subprocess inherits the current environment so it gets:
          - /opt/venvs/da3/bin on PATH (da3 venv Python + packages)
          - PYTHONPATH with Depth-Anything-3/src (for `depth_anything_3` imports)
          - PYTHONPATH with the da3_streaming directory is added automatically
            because Python prepends the script's own directory to sys.path.
        """
        session_dir = self._session_dir
        frames_dir  = self._frames_dir
        cmd = [
            self._da3_python,
            self._da3_script,
            '--image_dir', frames_dir,
            '--config',    self._da3_config,
            '--output_dir', session_dir,
        ]
        self.get_logger().info(
            f'{colorize("DA3 CMD", _MAGENTA, bold=True)}: ' + ' '.join(cmd)
        )

        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env=os.environ.copy(),
            )
            for line in proc.stdout:
                stripped = line.rstrip()
                if stripped:
                    self.get_logger().info(
                        f'{colorize("[DA3]", _MAGENTA, bold=True)} {stripped}'
                    )
            proc.wait()
            if proc.returncode != 0:
                raise RuntimeError(f'da3_streaming exited with code {proc.returncode}')
        except Exception as e:
            self.get_logger().error(
                f'{colorize("DA3-Streaming failed", _RED, bold=True)}: {e}'
            )
            with self._state_lock:
                self._state = State.ERROR
            return

        ply_path       = os.path.join(session_dir, 'pcd', 'combined_pcd.ply')
        poses_path     = os.path.join(session_dir, 'camera_poses.txt')
        intrinsics_path = os.path.join(session_dir, 'intrinsic.txt')

        try:
            self._publish_results(ply_path, poses_path, intrinsics_path)
        except Exception as e:
            self.get_logger().error(f'Result publishing failed: {e}')
            with self._state_lock:
                self._state = State.ERROR
            return

        with self._state_lock:
            self._state = State.DONE
        self.get_logger().info(
            f'{colorize("Processing complete", _GREEN, bold=True)}. '
            f'Results published in RViz.'
        )

    # ── Result publishing ──────────────────────────────────────────────────────

    def _publish_results(self, ply_path, poses_path, intrinsics_path):
        stamp  = self.get_clock().now().to_msg()
        header = Header(frame_id=self._frame_id, stamp=stamp)

        self.get_logger().info(f'Loading PLY: {ply_path}')
        points, colors = load_ply(ply_path, self._voxel)
        self._pc_pub.publish(build_pointcloud2(points, colors, header))
        self.get_logger().info(f'Published PointCloud2 ({len(points)} pts)')

        if os.path.exists(poses_path) and os.path.exists(intrinsics_path):
            poses      = read_camera_poses(poses_path)
            intrinsics = read_intrinsics(intrinsics_path)
            self._path_pub.publish(self._build_path(poses, header))
            self._axes_pub.publish(self._build_axes(poses, header))
            self._frustums_pub.publish(self._build_frustums(poses, intrinsics, header))
            self.get_logger().info(f'Published camera path/axes/frustums ({len(poses)} poses)')

    # ── Marker builders ────────────────────────────────────────────────────────

    def _build_path(self, poses, header):
        path = NavPath(header=header)
        for c2w in poses:
            ps = PoseStamped(header=header)
            ps.pose.position = make_point(c2w[:3, 3])
            qx, qy, qz, qw = rot_to_quat(c2w[:3, :3])
            ps.pose.orientation.x = float(qx)
            ps.pose.orientation.y = float(qy)
            ps.pose.orientation.z = float(qz)
            ps.pose.orientation.w = float(qw)
            path.poses.append(ps)
        return path

    def _build_axes(self, poses, header):
        markers = MarkerArray()
        for mid, (axis_dir, rgb) in enumerate([
            (np.array([1., 0., 0.]), (1., .1, .1)),   # X: red
            (np.array([0., 1., 0.]), (.1, 1., .1)),   # Y: green
            (np.array([0., 0., 1.]), (.1, .4, 1.)),   # Z: blue
        ]):
            m = Marker(header=header)
            m.ns, m.id, m.type, m.action = 'axes', mid, Marker.LINE_LIST, Marker.ADD
            m.scale.x = 0.005
            m.color.r, m.color.g, m.color.b, m.color.a = *rgb, 1.0
            for c2w in poses:
                origin = c2w[:3, 3]
                tip    = origin + c2w[:3, :3] @ (axis_dir * self._axis_len)
                m.points += [make_point(origin), make_point(tip)]
            markers.markers.append(m)
        return markers

    def _build_frustums(self, poses, intrinsics, header):
        m = Marker(header=header)
        m.ns, m.id, m.type, m.action = 'frustums', 0, Marker.LINE_LIST, Marker.ADD
        m.scale.x = 0.003
        m.color.r, m.color.g, m.color.b, m.color.a = 1., .75, .1, 1.
        d = self._frustum_depth
        for c2w, (fx, fy, cx, cy) in zip(poses, intrinsics):
            # Project image corners to 3-D at depth d in camera frame
            w, h = 2. * cx, 2. * cy
            corners_cam = np.array([
                [(0 - cx) / fx * d, (0 - cy) / fy * d, d],
                [(w - cx) / fx * d, (0 - cy) / fy * d, d],
                [(w - cx) / fx * d, (h - cy) / fy * d, d],
                [(0 - cx) / fx * d, (h - cy) / fy * d, d],
            ])
            corners = (c2w[:3, :3] @ corners_cam.T).T + c2w[:3, 3]
            origin  = c2w[:3, 3]
            for c in corners:
                m.points += [make_point(origin), make_point(c)]
            for a, b in [(0, 1), (1, 2), (2, 3), (3, 0)]:
                m.points += [make_point(corners[a]), make_point(corners[b])]
        return MarkerArray(markers=[m])

    # ── Status ─────────────────────────────────────────────────────────────────

    def _publish_status(self):
        with self._state_lock:
            state, saved = self._state, self._total_saved
        msgs = {
            State.IDLE:       'IDLE — call start_recording to begin',
            State.RECORDING:  f'RECORDING — {saved} frames saved',
            State.PROCESSING: f'PROCESSING — DA3-Streaming running on {saved} frames',
            State.DONE:       f'DONE — {saved} frames processed, results published',
            State.ERROR:      'ERROR — check node log for details',
        }
        self._status_pub.publish(String(data=msgs[state]))

    # ── Helpers ────────────────────────────────────────────────────────────────

    @staticmethod
    def _fail(response, msg):
        response.success = False
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FrameRecorderNode()
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
