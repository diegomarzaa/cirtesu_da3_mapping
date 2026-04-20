#!/usr/bin/env python3
"""
camera_poses_publisher_node — Publishes DA3 camera poses for RViz visualization.

Inputs
------
- camera_poses.txt: one flattened 4x4 C2W matrix per line
- intrinsic.txt: one line per frame with fx fy cx cy

Outputs
-------
- nav_msgs/Path
- visualization_msgs/MarkerArray with camera axes
- visualization_msgs/MarkerArray with camera frustums
"""
import sys

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray

from cirtesu_da3_mapping.camera_visualization import (
    build_axes_markers,
    build_frustum_markers,
    build_path,
    read_camera_poses,
    read_intrinsics,
)

_DEFAULT_POSES = ""
_DEFAULT_INTRINSICS = ""


class CameraPosesPublisher(Node):
    def __init__(self):
        super().__init__('camera_poses_publisher')

        self.declare_parameter('poses_path', _DEFAULT_POSES)
        self.declare_parameter('intrinsics_path', _DEFAULT_INTRINSICS)
        self.declare_parameter('frame_id', 'da3_world')
        self.declare_parameter('path_topic', '/cirtesu/camera_path')
        self.declare_parameter('axes_topic', '/cirtesu/camera_axes')
        self.declare_parameter('frustums_topic', '/cirtesu/camera_frustums')
        self.declare_parameter('publish_rate_hz', 0.0)
        self.declare_parameter('axis_length', 0.12)
        self.declare_parameter('axis_line_width', 0.01)
        self.declare_parameter('frustum_depth', 0.18)
        self.declare_parameter('frustum_line_width', 0.006)

        poses_path = self.get_parameter('poses_path').value
        intrinsics_path = self.get_parameter('intrinsics_path').value
        frame_id = self.get_parameter('frame_id').value
        path_topic = self.get_parameter('path_topic').value
        axes_topic = self.get_parameter('axes_topic').value
        frustums_topic = self.get_parameter('frustums_topic').value
        rate_hz = self.get_parameter('publish_rate_hz').value
        axis_length = float(self.get_parameter('axis_length').value)
        axis_line_width = float(self.get_parameter('axis_line_width').value)
        frustum_depth = float(self.get_parameter('frustum_depth').value)
        frustum_line_width = float(self.get_parameter('frustum_line_width').value)

        self.get_logger().info(f'Loading camera poses: {poses_path}')
        poses = read_camera_poses(poses_path)
        intrinsics = read_intrinsics(intrinsics_path)
        if len(poses) != len(intrinsics):
            raise RuntimeError(
                f'Pose/intrinsics length mismatch: {len(poses)} poses vs {len(intrinsics)} intrinsics'
            )

        self.get_logger().info(f'Loaded {len(poses)} camera poses')

        stamp = self.get_clock().now().to_msg()
        header = Header(frame_id=frame_id, stamp=stamp)
        self._path_msg = build_path(poses, header)
        self._axes_msg = build_axes_markers(poses, header, axis_length, axis_line_width)
        self._frustums_msg = build_frustum_markers(
            poses, intrinsics, header, frustum_depth, frustum_line_width
        )

        one_shot = rate_hz <= 0.0
        if one_shot:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            )
        else:
            qos = QoSProfile(depth=10)

        self._path_pub = self.create_publisher(Path, path_topic, qos)
        self._axes_pub = self.create_publisher(MarkerArray, axes_topic, qos)
        self._frustums_pub = self.create_publisher(MarkerArray, frustums_topic, qos)

        if one_shot:
            self._publish_all()
            self.get_logger().info(
                f'Published camera path/axes/frustums once '
                f'[nposes={len(poses)}, frame={frame_id}]'
            )
        else:
            self._timer = self.create_timer(1.0 / rate_hz, self._publish_all)
            self.get_logger().info(
                f'Publishing camera path/axes/frustums at {rate_hz:.1f} Hz '
                f'[nposes={len(poses)}, frame={frame_id}]'
            )

    def _publish_all(self):
        stamp = self.get_clock().now().to_msg()
        self._path_msg.header.stamp = stamp
        for pose_stamped in self._path_msg.poses:
            pose_stamped.header.stamp = stamp
        for marker in self._axes_msg.markers:
            marker.header.stamp = stamp
        for marker in self._frustums_msg.markers:
            marker.header.stamp = stamp

        self._path_pub.publish(self._path_msg)
        self._axes_pub.publish(self._axes_msg)
        self._frustums_pub.publish(self._frustums_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraPosesPublisher()
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
