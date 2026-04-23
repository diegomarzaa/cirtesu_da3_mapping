#!/usr/bin/env python3
"""Batch-incremental DA3 mapper.

This node captures incoming images and periodically rebuilds one global DA3
reconstruction from all images seen so far. If the global DA3 run is too large
for the GPU, it falls back to DA3-Streaming for the same frame set.
"""

from __future__ import annotations

import os
import sys
import threading
import time
from enum import Enum, auto
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

from cirtesu_da3_mapping.da3_batch_reconstructor import (
    BatchReconstructionConfig,
    Da3BatchReconstructor,
    prepare_input_dir,
)
from cirtesu_da3_mapping.image_io import (
    message_stamp_ns,
    resolve_image_topic_kind,
    save_image_message,
)
from cirtesu_da3_mapping.mapping_session import MappingSession
from cirtesu_da3_mapping.pointcloud_utils import build_pointcloud2

_IMAGE_TOPIC = "/image_raw/compressed"
_POINTCLOUD_TOPIC = "/cirtesu/map_pointcloud"
_FRAME_ID = "da3_world"


class State(Enum):
    IDLE = auto()
    RUNNING = auto()
    STOPPING = auto()
    ERROR = auto()


class MappingSimultaneousNode(Node):
    """Capture frames and republish a full DA3 map after each batch update."""

    def __init__(self):
        super().__init__("mapping_simultaneous")

        self.declare_parameter("auto_start", True)
        self.declare_parameter("image_topic", _IMAGE_TOPIC)
        self.declare_parameter("session_base_dir", str(Path.home() / "da3_sessions"))
        self.declare_parameter("da3_root_dir", "")
        self.declare_parameter("da3_cli", "da3")
        self.declare_parameter("da3_model_dir", "")
        self.declare_parameter("da3_streaming_dir", "")
        self.declare_parameter("streaming_config", "")
        self.declare_parameter("pointcloud_topic", _POINTCLOUD_TOPIC)
        self.declare_parameter("frame_id", _FRAME_ID)
        self.declare_parameter("target_save_fps", 30.0)
        self.declare_parameter("min_images_initial", 8)
        self.declare_parameter("min_new_images", 8)
        self.declare_parameter("min_seconds_between_runs", 30.0)
        self.declare_parameter("max_normal_images", 80)
        self.declare_parameter("process_res", 504)
        self.declare_parameter("num_max_points", 4_000_000)
        self.declare_parameter("conf_thresh_percentile", 10.0)
        self.declare_parameter("show_cameras", False)
        self.declare_parameter("fallback_to_streaming", True)
        self.declare_parameter("voxel_downsample", 0.0)
        self.declare_parameter("command_timeout_sec", 0.0)
        self.declare_parameter("save_depth_outputs", True)

        self._auto_start = bool(self.get_parameter("auto_start").value)
        self._image_topic = self.get_parameter("image_topic").value
        self._session_base_dir = self.get_parameter("session_base_dir").value
        self._da3_root_dir = self.get_parameter("da3_root_dir").value
        self._da3_cli = self.get_parameter("da3_cli").value
        self._da3_model_dir = self.get_parameter("da3_model_dir").value
        self._da3_streaming_dir = self.get_parameter("da3_streaming_dir").value
        self._streaming_config = self.get_parameter("streaming_config").value
        self._pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._target_save_fps = float(self.get_parameter("target_save_fps").value)
        self._min_images_initial = int(self.get_parameter("min_images_initial").value)
        self._min_new_images = int(self.get_parameter("min_new_images").value)
        self._min_seconds_between_runs = float(
            self.get_parameter("min_seconds_between_runs").value
        )
        self._max_normal_images = int(self.get_parameter("max_normal_images").value)
        self._process_res = int(self.get_parameter("process_res").value)
        self._num_max_points = int(self.get_parameter("num_max_points").value)
        self._conf_thresh_percentile = float(
            self.get_parameter("conf_thresh_percentile").value
        )
        self._show_cameras = bool(self.get_parameter("show_cameras").value)
        self._fallback_to_streaming = bool(self.get_parameter("fallback_to_streaming").value)
        self._voxel_size = float(self.get_parameter("voxel_downsample").value)
        self._command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self._save_depth_outputs = bool(self.get_parameter("save_depth_outputs").value)
        self._min_period_ns = (
            int(1e9 / self._target_save_fps) if self._target_save_fps > 0.0 else 0
        )

        self._validate_config()
        self._reconstructor = Da3BatchReconstructor(
            self._batch_config(),
            log_info=self.get_logger().info,
            log_warning=self.get_logger().warning,
        )

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._pc_pub = self.create_publisher(PointCloud2, self._pointcloud_topic, latched)
        self._status_pub = self.create_publisher(String, "~/status", 10)

        self._state = State.IDLE
        self._state_lock = threading.Lock()
        self._capture_lock = threading.Lock()
        self._frames_cv = threading.Condition()
        self._stop_event = threading.Event()
        self._force_event = threading.Event()
        self._worker: threading.Thread | None = None
        self._img_sub = None
        self._topic_probe_timer = None
        self._last_saved_ns = 0
        self._last_topic_wait_log_ns = 0
        self._last_processed_count = 0
        self._last_run_monotonic = 0.0
        self._run_index = 0
        self._last_output = "none"
        self._last_error = ""
        self._session = MappingSession(self._session_base_dir, debug_save=False)

        self.create_timer(1.0, self._publish_status)
        self.create_service(Trigger, "~/start", self._srv_start)
        self.create_service(Trigger, "~/stop", self._srv_stop)
        self.create_service(Trigger, "~/run_now", self._srv_run_now)

        self.get_logger().info(
            "Mapping simultaneous ready."
            f"\n  auto_start: {self._auto_start}"
            f"\n  image_topic: {self._image_topic}"
            f"\n  da3_root_dir: {self._da3_root_dir}"
            f"\n  da3_model_dir: {self._da3_model_dir}"
            f"\n  da3_streaming_dir: {self._da3_streaming_dir}"
            f"\n  streaming_config: {self._streaming_config}"
            f"\n  min_images_initial: {self._min_images_initial}"
            f"\n  min_new_images: {self._min_new_images}"
            f"\n  min_seconds_between_runs: {self._min_seconds_between_runs}"
            f"\n  max_normal_images: {self._max_normal_images}"
            f"\n  process_res: {self._process_res}"
            f"\n  num_max_points: {self._num_max_points}"
            f"\n  fallback_to_streaming: {self._fallback_to_streaming}"
        )

        if self._auto_start:
            self._start_session()

    def _batch_config(self) -> BatchReconstructionConfig:
        return BatchReconstructionConfig(
            da3_root_dir=self._da3_root_dir,
            da3_cli=self._da3_cli,
            da3_model_dir=self._da3_model_dir,
            da3_streaming_dir=self._da3_streaming_dir,
            streaming_config=self._streaming_config,
            max_normal_images=self._max_normal_images,
            process_res=self._process_res,
            num_max_points=self._num_max_points,
            conf_thresh_percentile=self._conf_thresh_percentile,
            show_cameras=self._show_cameras,
            fallback_to_streaming=self._fallback_to_streaming,
            voxel_downsample=self._voxel_size,
            command_timeout_sec=self._command_timeout_sec,
            save_depth_outputs=self._save_depth_outputs,
        )

    def _validate_config(self) -> None:
        missing = []
        if self._min_images_initial < 1:
            missing.append("min_images_initial must be >= 1")
        if self._min_new_images < 1:
            missing.append("min_new_images must be >= 1")
        if missing:
            raise RuntimeError("Invalid mapping_simultaneous configuration:\n  - " + "\n  - ".join(missing))

    def _srv_start(self, _request, response):
        try:
            session_dir = self._start_session()
        except RuntimeError as exc:
            return self._fail(response, str(exc))
        response.success = True
        response.message = f"Running. Session: {session_dir}"
        return response

    def _srv_stop(self, _request, response):
        with self._state_lock:
            if self._state != State.RUNNING:
                return self._fail(response, f"Not running (state={self._state.name}).")
            self._state = State.STOPPING

        self._destroy_image_subscription()
        self._destroy_topic_probe_timer()
        self._stop_event.set()
        with self._frames_cv:
            self._frames_cv.notify_all()

        if self._worker is not None:
            self._worker.join()
            self._worker = None

        with self._state_lock:
            self._state = State.IDLE
        response.success = True
        response.message = "Stopped mapping_simultaneous."
        return response

    def _srv_run_now(self, _request, response):
        with self._state_lock:
            if self._state != State.RUNNING:
                return self._fail(response, f"Not running (state={self._state.name}).")
        self._force_event.set()
        with self._frames_cv:
            self._frames_cv.notify_all()
        response.success = True
        response.message = "Scheduled immediate reconstruction."
        return response

    def _start_session(self) -> str:
        with self._state_lock:
            if self._state == State.RUNNING:
                raise RuntimeError("Already running.")
            session_dir = self._session.start()
            self._state = State.RUNNING
            self._last_saved_ns = 0
            self._last_processed_count = 0
            self._last_run_monotonic = 0.0
            self._run_index = 0
            self._last_error = ""
            self._last_output = "none"
            self._stop_event.clear()
            self._force_event.clear()

        self._subscribe_or_wait_for_image_topic()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(f"RUNNING session={session_dir}")
        return session_dir

    def _subscribe_or_wait_for_image_topic(self) -> None:
        if self._img_sub is not None:
            return

        kind = resolve_image_topic_kind(self.get_topic_names_and_types(), self._image_topic)
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

    def _destroy_image_subscription(self) -> None:
        if self._img_sub is not None:
            self.destroy_subscription(self._img_sub)
            self._img_sub = None

    def _destroy_topic_probe_timer(self) -> None:
        if self._topic_probe_timer is not None:
            self.destroy_timer(self._topic_probe_timer)
            self._topic_probe_timer = None

    def _maybe_log_topic_wait(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_topic_wait_log_ns < 5_000_000_000:
            return
        self._last_topic_wait_log_ns = now_ns
        self.get_logger().info(
            f"Waiting for {self._image_topic} as Image or CompressedImage..."
        )

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

    def _worker_loop(self) -> None:
        while not self._stop_event.is_set():
            frame_paths = self._wait_for_processable_frames()
            if not frame_paths:
                continue
            try:
                self._process_snapshot(frame_paths)
            except Exception as exc:
                self._set_error(str(exc))
                return

    def _wait_for_processable_frames(self) -> list[str]:
        while not self._stop_event.is_set():
            with self._frames_cv:
                count = self._session.frame_count()
                now = time.monotonic()
                enough_initial = count >= self._min_images_initial
                enough_new = count >= self._last_processed_count + self._min_new_images
                enough_time = (
                    self._last_run_monotonic <= 0.0
                    or now - self._last_run_monotonic >= self._min_seconds_between_runs
                )
                forced = self._force_event.is_set()
                if enough_initial and (forced or (enough_new and enough_time)):
                    self._force_event.clear()
                    return self._session.frame_paths_snapshot()
                self._frames_cv.wait(timeout=0.5)
        return []

    def _process_snapshot(self, frame_paths: list[str]) -> None:
        count = len(frame_paths)
        session_dir = self._session.session_dir
        if session_dir is None:
            raise RuntimeError("No active session directory.")

        run_name = f"recon_{self._run_index:04d}_{count:06d}"
        self._run_index += 1
        run_dir = os.path.join(session_dir, "reconstructions", run_name)
        input_dir = os.path.join(run_dir, "input_images")
        os.makedirs(run_dir, exist_ok=True)
        prepare_input_dir(frame_paths, input_dir)

        self.get_logger().info(
            f"Reconstructing {count} images into {run_dir} "
            f"(normal_limit={self._max_normal_images})"
        )

        result = self._reconstructor.reconstruct(input_dir, run_dir)
        points, colors = result.points, result.colors
        self._last_output = f"{result.mode}:{count}"

        header = Header(
            frame_id=self._frame_id,
            stamp=self.get_clock().now().to_msg(),
        )
        msg = build_pointcloud2(points, colors, header)
        self._pc_pub.publish(msg)
        self._last_processed_count = count
        self._last_run_monotonic = time.monotonic()
        self.get_logger().info(
            f"Published full reconstruction from {count} images "
            f"on {self._pointcloud_topic} [{msg.width} pts, frame={self._frame_id}]"
            f"\nDepth outputs: {result.depth_dir or 'disabled'}"
        )

    def _publish_status(self) -> None:
        n_frames = self._session.frame_count()
        msg = String()
        msg.data = (
            f"state={self._state.name} frames={n_frames} "
            f"last_processed={self._last_processed_count} "
            f"last_output={self._last_output} last_error={self._last_error}"
        )
        self._status_pub.publish(msg)

    def _set_error(self, message: str) -> None:
        self._last_error = message
        with self._state_lock:
            self._state = State.ERROR
        self.get_logger().error(message)

    def _fail(self, response, message: str):
        response.success = False
        response.message = message
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MappingSimultaneousNode()
    except Exception as exc:
        print(f"[FATAL] {exc}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
