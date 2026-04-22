#!/usr/bin/env python3
"""
glb_publisher_node - Publishes a DA3 .glb point cloud as sensor_msgs/PointCloud2.

DA3 exports its colored reconstruction as a trimesh PointCloud inside the GLB.
Camera wireframes are exported as Path3D geometries, so this loader keeps only
PointCloud geometries by default.
"""
from __future__ import annotations

import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from cirtesu_da3_mapping.pointcloud_utils import (
    build_pointcloud2,
    voxel_downsample,
)


def load_da3_glb(path: str, voxel_size: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Load point geometries from a DA3 GLB as float32 XYZ and uint8 RGB."""
    import trimesh

    scene = trimesh.load(path, force="scene")
    point_sets = []
    color_sets = []

    for geom in scene.geometry.values():
        if not isinstance(geom, trimesh.points.PointCloud):
            continue

        points = np.asarray(geom.vertices, dtype=np.float32)
        if len(points) == 0:
            continue

        vertex_colors = getattr(geom.visual, "vertex_colors", None)
        if vertex_colors is None or len(vertex_colors) != len(points):
            colors = np.full((len(points), 3), 255, dtype=np.uint8)
        else:
            colors = np.asarray(vertex_colors[:, :3], dtype=np.uint8)

        point_sets.append(points)
        color_sets.append(colors)

    if not point_sets:
        raise RuntimeError(f"GLB has no PointCloud geometry: {path}")

    points = np.concatenate(point_sets, axis=0)
    colors = np.concatenate(color_sets, axis=0)
    points, colors = voxel_downsample(points, colors, voxel_size)
    return points, colors


class GlbPublisher(Node):
    def __init__(self):
        super().__init__("glb_publisher")

        self.declare_parameter("glb_path", "")
        self.declare_parameter("frame_id", "da3_world")
        self.declare_parameter("topic", "/cirtesu/map_pointcloud")
        self.declare_parameter("publish_rate_hz", 0.0)
        self.declare_parameter("voxel_downsample", 0.0)

        glb_path = self.get_parameter("glb_path").value
        frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("topic").value
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        voxel = float(self.get_parameter("voxel_downsample").value)

        self.get_logger().info(f"Loading GLB: {glb_path}")
        points, colors = load_da3_glb(glb_path, voxel)
        self.get_logger().info(
            f"Loaded {len(points)} points "
            f"(voxel_downsample={voxel if voxel > 0 else 'off'})"
        )

        header = Header(
            frame_id=frame_id,
            stamp=self.get_clock().now().to_msg(),
        )
        self._msg = build_pointcloud2(points, colors, header)

        one_shot = rate_hz <= 0.0
        if one_shot:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            )
        else:
            qos = QoSProfile(depth=10)

        self._pub = self.create_publisher(PointCloud2, topic, qos)

        if one_shot:
            self._pub.publish(self._msg)
            self.get_logger().info(
                f"Published once on {topic} "
                f"[{self._msg.width} pts, frame={frame_id}, transient_local]"
            )
        else:
            self._timer = self.create_timer(1.0 / rate_hz, self._publish_cb)
            self.get_logger().info(
                f"Publishing {rate_hz:.1f} Hz on {topic} "
                f"[{self._msg.width} pts, frame={frame_id}]"
            )

    def _publish_cb(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GlbPublisher()
    except Exception as e:
        print(f"[FATAL] {e}", file=sys.stderr)
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
