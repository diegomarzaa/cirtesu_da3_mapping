"""Unified ROS2 mapper node for incremental and on-stop DA3 processing."""

from __future__ import annotations

import os
import shutil
import sys
import threading
from enum import Enum, auto
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

from cirtesu_da3_mapping.camera_visualization import build_path
from cirtesu_da3_mapping.da3_engine import ChunkResult, DA3Engine
from cirtesu_da3_mapping.image_io import (
    message_stamp_ns,
    resolve_image_topic_kind,
    save_image_message,
)
from cirtesu_da3_mapping.mapping_session import (
    ChunkWindow,
    MappingSession,
    iter_chunk_windows,
)
from cirtesu_da3_mapping.pointcloud_utils import (
    build_pointcloud2,
    voxel_downsample,
)

_IMAGE_TOPIC = "/image_raw/compressed"
_POINTCLOUD_TOPIC = "/cirtesu/map_pointcloud"
_PATH_TOPIC = "/cirtesu/camera_path"
_FRAME_ID = "da3_world"
_PROCESSING_MODES = ("incremental", "on_stop")


class State(Enum):
    IDLE = auto()
    RUNNING = auto()
    PROCESSING = auto()
    DONE = auto()
    ERROR = auto()


class UnifiedMappingNode(Node):
    """Capture frames, process DA3 chunks, and publish a map."""

    def __init__(self):
        super().__init__("da3_mapper")

        self.declare_parameter("processing_mode", "incremental")
        self.declare_parameter("image_topic", _IMAGE_TOPIC)
        self.declare_parameter("session_base_dir", str(Path.home() / "da3_sessions"))
        self.declare_parameter("da3_streaming_dir", "")
        self.declare_parameter("da3_src_dir", "")
        self.declare_parameter("da3_config", "")
        self.declare_parameter("pointcloud_topic", _POINTCLOUD_TOPIC)
        self.declare_parameter("path_topic", _PATH_TOPIC)
        self.declare_parameter("frame_id", _FRAME_ID)
        self.declare_parameter("target_save_fps", 1.0)
        self.declare_parameter("debug_save", False)
        self.declare_parameter("voxel_downsample", 0.01)
        self.declare_parameter("publish_accumulated", True)
        # self.declare_parameter("max_publish_points", 900000)

        self._processing_mode = self.get_parameter("processing_mode").value
        if self._processing_mode not in _PROCESSING_MODES:
            raise ValueError(
                f"processing_mode must be one of {_PROCESSING_MODES}, "
                f"got {self._processing_mode!r}"
            )

        self._image_topic = self.get_parameter("image_topic").value
        self._session_base_dir = self.get_parameter("session_base_dir").value
        self._da3_streaming_dir = self.get_parameter("da3_streaming_dir").value
        self._da3_src_dir = self.get_parameter("da3_src_dir").value
        self._da3_config = self.get_parameter("da3_config").value
        self._pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self._path_topic = self.get_parameter("path_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._target_save_fps = float(self.get_parameter("target_save_fps").value)
        self._debug_save = bool(self.get_parameter("debug_save").value)
        self._voxel_size = float(self.get_parameter("voxel_downsample").value)
        self._publish_accumulated = bool(self.get_parameter("publish_accumulated").value)
        # self._max_publish_points = int(self.get_parameter("max_publish_points").value)
        self._min_period_ns = (
            int(1e9 / self._target_save_fps) if self._target_save_fps > 0.0 else 0
        )

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._pc_pub = self.create_publisher(PointCloud2, self._pointcloud_topic, latched)
        self._path_pub = self.create_publisher(NavPath, self._path_topic, latched)
        self._status_pub = self.create_publisher(String, "~/status", 10)

        self._state = State.IDLE
        self._state_lock = threading.Lock()
        self._capture_lock = threading.Lock()
        self._frames_cv = threading.Condition()
        self._stop_event = threading.Event()
        self._worker: threading.Thread | None = None
        self._img_sub = None
        self._topic_probe_timer = None
        self._image_kind: str | None = None
        self._last_saved_ns = 0
        self._last_topic_wait_log_ns = 0

        self._session = MappingSession(self._session_base_dir, self._debug_save)

        self.create_timer(1.0, self._publish_status)
        self.create_service(Trigger, "~/start", self._srv_start)
        self.create_service(Trigger, "~/stop", self._srv_stop)

        self.get_logger().info("Loading DA3 model...")
        self._validate_required_config()
        self._engine = DA3Engine(
            config_path=self._da3_config,
            da3_streaming_dir=self._da3_streaming_dir,
            da3_src_dir=self._da3_src_dir,
        )
        self.get_logger().info(
            "DA3 model ready "
            f"(chunk_size={self._engine.chunk_size}, "
            f"overlap={self._engine.overlap}, device={self._engine.device})."
            f"Parameters:"
            f"\n  target_save_fps: {self._target_save_fps}"
            f"\n  debug_save: {self._debug_save}"
            f"\n  voxel_downsample: {self._voxel_size}"
            f"\n  publish_accumulated: {self._publish_accumulated}"
            f"\n  image_topic: {self._image_topic}"
            f"\n  processing_mode: {self._processing_mode}"
            f"\n  session_base_dir: {self._session_base_dir}"
            f"\n  da3_streaming_dir: {self._da3_streaming_dir}"
            f"\n  da3_src_dir: {self._da3_src_dir}"
            f"\n  da3_config: {self._da3_config}"
            # f"\n  max_publish_points: {self._max_publish_points}"
        )
        self.get_logger().info(
            f"Mode={self._processing_mode}, image_topic={self._image_topic}. "
            "Call /da3_mapper/start to begin."
        )
        
    def _validate_required_config(self) -> None:
        missing = []
        if not self._da3_streaming_dir:
            missing.append("da3_streaming_dir is empty")
        elif not os.path.isdir(self._da3_streaming_dir):
            missing.append(f"da3_streaming_dir does not exist: {self._da3_streaming_dir}")

        if not self._da3_src_dir:
            missing.append("da3_src_dir is empty")
        elif not os.path.isdir(self._da3_src_dir):
            missing.append(f"da3_src_dir does not exist: {self._da3_src_dir}")

        if not self._da3_config:
            missing.append("da3_config is empty")
        elif not os.path.isfile(self._da3_config):
            missing.append(f"da3_config does not exist: {self._da3_config}")

        if shutil.which("python3") is None:
            missing.append("python3 is not available in PATH")

        if missing:
            details = "\n  - ".join(missing)
            raise RuntimeError(
                "Invalid DA3 mapping configuration. Fix launch/mapping_defaults.yaml "
                "or pass launch arguments.\n  - "
                f"{details}"
            )

    def _srv_start(self, _request, response):
        with self._state_lock:
            if self._state in (State.RUNNING, State.PROCESSING):
                return self._fail(response, f"Cannot start while {self._state.name}.")
            session_dir = self._session.start()
            self._engine.reset()
            self._last_saved_ns = 0
            self._image_kind = None
            self._stop_event.clear()
            self._state = State.RUNNING

        self._subscribe_or_wait_for_image_topic()
        if self._processing_mode == "incremental":
            self._worker = threading.Thread(target=self._incremental_worker, daemon=True)
            self._worker.start()

        response.success = True
        response.message = f"Running. Session: {session_dir}"
        self.get_logger().info(f"RUNNING session={session_dir}")
        return response

    def _srv_stop(self, _request, response):
        with self._state_lock:
            if self._state != State.RUNNING:
                return self._fail(response, f"Not running (state={self._state.name}).")
            if self._session.frame_count() < 1:
                return self._fail(response, "No frames saved yet.")

        self._destroy_image_subscription()
        self._destroy_topic_probe_timer()
        self._stop_event.set()
        with self._frames_cv:
            self._frames_cv.notify_all()

        if self._processing_mode == "incremental":
            if self._worker is not None:
                self._worker.join()
                self._worker = None
            with self._state_lock:
                failed = self._state == State.ERROR
                if not failed:
                    self._state = State.DONE
            response.success = not failed
            response.message = (
                "Stopped incremental mapping."
                if not failed else
                "Stopped incremental mapping, but final processing failed."
            )
        else:
            with self._state_lock:
                self._state = State.PROCESSING
            self._worker = threading.Thread(target=self._on_stop_worker, daemon=True)
            self._worker.start()
            response.success = True
            response.message = "Stopped capture. Processing all chunks."

        self.get_logger().info(response.message)
        return response

    def _subscribe_or_wait_for_image_topic(self) -> None:
        if self._img_sub is not None:
            return

        kind = resolve_image_topic_kind(
            self.get_topic_names_and_types(),
            self._image_topic,
        )
        if kind is None:
            self._maybe_log_topic_wait()
            if self._topic_probe_timer is None:
                self._topic_probe_timer = self.create_timer(0.5, self._topic_probe_cb)
            return

        msg_type = CompressedImage if kind == "compressed" else Image
        self._img_sub = self.create_subscription(
            msg_type,
            self._image_topic,
            self._image_callback,
            10,
        )
        self._image_kind = kind
        self._destroy_topic_probe_timer()
        self.get_logger().info(
            f"Subscribed to {self._image_topic} as {msg_type.__name__}."
        )

    def _topic_probe_cb(self) -> None:
        with self._state_lock:
            if self._state != State.RUNNING:
                self._destroy_topic_probe_timer()
                return
        self._subscribe_or_wait_for_image_topic()

    def _maybe_log_topic_wait(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_topic_wait_log_ns < 5_000_000_000:
            return
        self._last_topic_wait_log_ns = now_ns
        self.get_logger().info(
            f"Waiting for {self._image_topic} as Image or CompressedImage..."
        )

    def _destroy_image_subscription(self) -> None:
        if self._img_sub is not None:
            self.destroy_subscription(self._img_sub)
            self._img_sub = None

    def _destroy_topic_probe_timer(self) -> None:
        if self._topic_probe_timer is not None:
            self.destroy_timer(self._topic_probe_timer)
            self._topic_probe_timer = None

    def _image_callback(self, msg) -> None:
        with self._state_lock:
            if self._state != State.RUNNING:
                return

        with self._capture_lock:
            stamp_ns = message_stamp_ns(msg, self.get_clock().now().nanoseconds)
            if (
                self._min_period_ns > 0
                and self._last_saved_ns > 0
                and (stamp_ns - self._last_saved_ns) < self._min_period_ns
            ):
                return
            self._last_saved_ns = stamp_ns

            frames_dir = self._session.frames_dir
            if frames_dir is None:
                self.get_logger().warning("Frame received before session directory exists.")
                return

            try:
                index = self._session.frame_count()
                path = save_image_message(msg, frames_dir, index)
                frame_count = self._session.add_frame(path)
            except Exception as exc:
                self.get_logger().warning(f"Image save failed: {exc}")
                return

        with self._frames_cv:
            self._frames_cv.notify_all()
        self.get_logger().info(
            f"Saved frame #{frame_count - 1:06d} ({os.path.basename(path)})"
        )

    def _incremental_worker(self) -> None:
        chunk_size = self._engine.chunk_size
        overlap = self._engine.overlap
        step = chunk_size - overlap
        next_start = 0

        while not self._stop_event.is_set():
            with self._frames_cv:
                while (
                    not self._stop_event.is_set()
                    and self._session.frame_count() < next_start + chunk_size
                ):
                    self._frames_cv.wait(timeout=0.25)
                if self._stop_event.is_set():
                    break
                frame_paths = self._session.frame_paths_snapshot()

            window = ChunkWindow(
                start=next_start,
                paths=frame_paths[next_start:next_start + chunk_size],
                partial=False,
            )
            if not self._process_window(window, publish_now=True):
                return
            next_start += step

        for window in iter_chunk_windows(
            self._session.frame_paths_snapshot(),
            chunk_size,
            overlap,
            start_index=next_start,
            include_partial=True,
        ):
            if not self._process_window(window, publish_now=True):
                return

        self.get_logger().info("Incremental worker exited.")

    def _on_stop_worker(self) -> None:
        self._engine.reset()
        windows = list(iter_chunk_windows(
            self._session.frame_paths_snapshot(),
            self._engine.chunk_size,
            self._engine.overlap,
            include_partial=True,
        ))
        if not windows:
            self._set_error("No processable chunks were found.")
            return

        for window in windows:
            if not self._process_window(window, publish_now=False):
                return

        self._publish_accumulated_outputs()
        with self._state_lock:
            if self._state != State.ERROR:
                self._state = State.DONE
        self.get_logger().info("On-stop processing complete.")

    def _process_window(self, window: ChunkWindow, publish_now: bool) -> bool:
        label = "partial chunk" if window.partial else "chunk"
        self.get_logger().info(
            f"Processing {label} starting at frame {window.start} "
            f"({len(window.paths)} frames)."
        )
        try:
            result = self._engine.process_chunk(window.paths)
        except Exception as exc:
            self._set_error(f"DA3 processing failed at frame {window.start}: {exc}")
            return False

        self._integrate_result(result, publish_now=publish_now)
        return True

    def _integrate_result(self, result: ChunkResult, publish_now: bool) -> None:
        n_chunks, n_points, n_poses = self._session.integrate(result)
        if self._debug_save:
            self._session.debug_dump(result)

        if publish_now:
            header = self._make_header()
            if self._publish_accumulated:
                self._publish_accumulated_pointcloud(header)
            else:
                self._publish_pointcloud(result.points, result.colors, header)
            self._publish_accumulated_path(header)

        self.get_logger().info(
            f"Integrated chunk {result.chunk_idx}: "
            f"chunks={n_chunks}, points={n_points}, poses={n_poses}"
        )

    def _publish_accumulated_outputs(self) -> None:
        header = self._make_header()
        self._publish_accumulated_pointcloud(header)
        self._publish_accumulated_path(header)

    def _publish_accumulated_pointcloud(self, header: Header) -> None:
        points, colors = self._session.accumulated_points_colors()
        if points is None or colors is None:
            return
        self._publish_pointcloud(points, colors, header)

    def _publish_pointcloud(
        self,
        points,
        colors,
        header: Header,
    ) -> None:
        if len(points) == 0:
            return
        points, colors = voxel_downsample(points, colors, self._voxel_size)
        # if self._max_publish_points > 0 and len(points) > self._max_publish_points:
        #     idx = np.random.choice(len(points), self._max_publish_points, replace=False)
        #     points = points[idx]
        #     colors = colors[idx]
        self._pc_pub.publish(build_pointcloud2(points, colors, header))

    def _publish_accumulated_path(self, header: Header) -> None:
        poses = self._session.poses_snapshot()
        if not poses:
            return
        self._path_pub.publish(build_path(poses, header))

    def _make_header(self) -> Header:
        return Header(frame_id=self._frame_id, stamp=self.get_clock().now().to_msg())

    def _publish_status(self) -> None:
        with self._state_lock:
            state = self._state
        n_chunks, n_points, n_poses = self._session.stats()
        n_frames = self._session.frame_count()
        topic_state = self._image_kind or "waiting"

        messages = {
            State.IDLE: "IDLE - call /da3_mapper/start to begin",
            State.RUNNING: (
                f"RUNNING - mode={self._processing_mode}, image={topic_state}, "
                f"frames={n_frames}, chunks={n_chunks}, "
                f"points={n_points}, poses={n_poses}"
            ),
            State.PROCESSING: (
                f"PROCESSING - frames={n_frames}, chunks={n_chunks}, "
                f"points={n_points}, poses={n_poses}"
            ),
            State.DONE: (
                f"DONE - frames={n_frames}, chunks={n_chunks}, "
                f"points={n_points}, poses={n_poses}"
            ),
            State.ERROR: "ERROR - check node log for details",
        }
        self._status_pub.publish(String(data=messages[state]))

    def _set_error(self, message: str) -> None:
        self.get_logger().error(message)
        with self._state_lock:
            self._state = State.ERROR
        self._stop_event.set()
        with self._frames_cv:
            self._frames_cv.notify_all()

    @staticmethod
    def _fail(response, message: str):
        response.success = False
        response.message = message
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = UnifiedMappingNode()
    except Exception as exc:
        print(f"[FATAL] {exc}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
