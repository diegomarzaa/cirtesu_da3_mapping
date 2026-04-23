#!/usr/bin/env python3
"""Run DA3 on an existing image folder and publish the result as PointCloud2."""

from __future__ import annotations

import os
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

from cirtesu_da3_mapping.da3_batch_reconstructor import (
    BatchReconstructionConfig,
    Da3BatchReconstructor,
    find_image_files,
)
from cirtesu_da3_mapping.pointcloud_utils import build_pointcloud2


_POINTCLOUD_TOPIC = "/cirtesu/map_pointcloud"
_FRAME_ID = "da3_world"


class MappingFolderNode(Node):
    """One-shot folder reconstruction node with an optional rerun service."""

    def __init__(self):
        super().__init__("mapping_folder")

        self.declare_parameter("image_dir", "")
        self.declare_parameter("output_dir", "/tmp/da3_folder_mapping")
        self.declare_parameter("process_on_start", True)
        self.declare_parameter("force_streaming", False)
        self.declare_parameter("da3_root_dir", "")
        self.declare_parameter("da3_cli", "da3")
        self.declare_parameter("da3_model_dir", "")
        self.declare_parameter("da3_streaming_dir", "")
        self.declare_parameter("streaming_config", "")
        self.declare_parameter("pointcloud_topic", _POINTCLOUD_TOPIC)
        self.declare_parameter("frame_id", _FRAME_ID)
        self.declare_parameter("max_normal_images", 80)
        self.declare_parameter("process_res", 504)
        self.declare_parameter("num_max_points", 4_000_000)
        self.declare_parameter("conf_thresh_percentile", 10.0)
        self.declare_parameter("show_cameras", False)
        self.declare_parameter("fallback_to_streaming", True)
        self.declare_parameter("voxel_downsample", 0.0)
        self.declare_parameter("command_timeout_sec", 0.0)
        self.declare_parameter("save_depth_outputs", True)

        self._image_dir = self.get_parameter("image_dir").value
        self._output_dir = self.get_parameter("output_dir").value
        self._process_on_start = bool(self.get_parameter("process_on_start").value)
        self._force_streaming = bool(self.get_parameter("force_streaming").value)
        self._pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._run_index = 0
        self._running = False
        self._last_status = "idle"
        self._last_output = "none"
        self._lock = threading.Lock()

        self._validate_folder()
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
        self._pc_pub = self.create_publisher(
            PointCloud2,
            self._pointcloud_topic,
            latched,
        )
        self._status_pub = self.create_publisher(String, "~/status", 10)
        self.create_service(Trigger, "~/run", self._srv_run)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            "Mapping folder ready."
            f"\n  image_dir: {self._image_dir}"
            f"\n  output_dir: {self._output_dir}"
            f"\n  process_on_start: {self._process_on_start}"
            f"\n  force_streaming: {self._force_streaming}"
        )

        if self._process_on_start:
            self._start_reconstruction()

    def _batch_config(self) -> BatchReconstructionConfig:
        return BatchReconstructionConfig(
            da3_root_dir=self.get_parameter("da3_root_dir").value,
            da3_cli=self.get_parameter("da3_cli").value,
            da3_model_dir=self.get_parameter("da3_model_dir").value,
            da3_streaming_dir=self.get_parameter("da3_streaming_dir").value,
            streaming_config=self.get_parameter("streaming_config").value,
            max_normal_images=int(self.get_parameter("max_normal_images").value),
            process_res=int(self.get_parameter("process_res").value),
            num_max_points=int(self.get_parameter("num_max_points").value),
            conf_thresh_percentile=float(
                self.get_parameter("conf_thresh_percentile").value
            ),
            show_cameras=bool(self.get_parameter("show_cameras").value),
            fallback_to_streaming=bool(self.get_parameter("fallback_to_streaming").value),
            voxel_downsample=float(self.get_parameter("voxel_downsample").value),
            command_timeout_sec=float(self.get_parameter("command_timeout_sec").value),
            save_depth_outputs=bool(self.get_parameter("save_depth_outputs").value),
        )

    def _validate_folder(self) -> None:
        if not self._image_dir or not os.path.isdir(self._image_dir):
            raise RuntimeError(f"image_dir does not exist: {self._image_dir}")
        image_count = len(find_image_files(self._image_dir))
        if image_count < 1:
            raise RuntimeError(f"image_dir has no png/jpg/jpeg images: {self._image_dir}")

    def _srv_run(self, _request, response):
        try:
            started = self._start_reconstruction()
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
            return response

        response.success = True
        response.message = "Reconstruction started." if started else "Already running."
        return response

    def _start_reconstruction(self) -> bool:
        with self._lock:
            if self._running:
                return False
            self._running = True
            self._last_status = "running"

        thread = threading.Thread(target=self._run_once, daemon=True)
        thread.start()
        return True

    def _run_once(self) -> None:
        try:
            image_count = len(find_image_files(self._image_dir))
            run_name = time.strftime(f"run_%Y%m%d_%H%M%S_{self._run_index:04d}")
            self._run_index += 1
            run_dir = os.path.join(self._output_dir, run_name)
            self.get_logger().info(
                f"Reconstructing folder with {image_count} images into {run_dir}"
            )
            result = self._reconstructor.reconstruct(
                self._image_dir,
                run_dir,
                force_streaming=self._force_streaming,
            )

            header = Header(
                frame_id=self._frame_id,
                stamp=self.get_clock().now().to_msg(),
            )
            msg = build_pointcloud2(result.points, result.colors, header)
            self._pc_pub.publish(msg)
            self._last_output = result.output_path
            self._last_status = (
                f"done mode={result.mode} images={result.image_count} "
                f"points={msg.width} depth_dir={result.depth_dir}"
            )
            self.get_logger().info(
                f"Published {result.mode} reconstruction on {self._pointcloud_topic} "
                f"[{msg.width} pts, frame={self._frame_id}]"
                f"\nDepth outputs: {result.depth_dir or 'disabled'}"
            )
        except Exception as exc:
            self._last_status = f"error: {exc}"
            self.get_logger().error(str(exc))
        finally:
            with self._lock:
                self._running = False

    def _publish_status(self) -> None:
        msg = String()
        msg.data = f"status={self._last_status} output={self._last_output}"
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MappingFolderNode()
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
