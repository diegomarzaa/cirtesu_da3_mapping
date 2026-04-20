#!/usr/bin/env python3
"""
ply_publisher_node — Publishes a .ply pointcloud as sensor_msgs/PointCloud2.

Coordinate frames
-----------------
DA3-Streaming writes PLYs in the *OpenCV world* convention (X=right, Y=down,
Z=forward) inherited from the first camera. We publish those points unchanged
with frame_id=`da3_world`. The launch file broadcasts a static TF
`map → da3_world` that rotates it into ROS REP-103 (X=forward, Y=left, Z=up),
so RViz2 with Fixed Frame=`map` shows the scene upright.
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from cirtesu_da3_mapping.pointcloud_utils import build_pointcloud2, load_ply


class PlyPublisher(Node):
    def __init__(self):
        super().__init__('ply_publisher')

        self.declare_parameter('ply_path', '')
        self.declare_parameter('frame_id', 'da3_world')
        self.declare_parameter('topic', '/cirtesu/map_pointcloud')
        self.declare_parameter('publish_rate_hz', 0.0)
        self.declare_parameter('voxel_downsample', 0.0)

        ply_path = self.get_parameter('ply_path').value
        frame_id = self.get_parameter('frame_id').value
        topic    = self.get_parameter('topic').value
        rate_hz  = self.get_parameter('publish_rate_hz').value
        voxel    = self.get_parameter('voxel_downsample').value

        # PLY is loaded once at startup — re-reading on every tick would push
        # the same hundreds of MB over DDS for nothing.
        self.get_logger().info(f'Loading PLY: {ply_path}')
        points, colors = load_ply(ply_path, voxel)
        self.get_logger().info(
            f'Loaded {len(points)} points '
            f'(voxel_downsample={voxel if voxel > 0 else "off"})'
        )

        header = Header(
            frame_id=frame_id,
            stamp=self.get_clock().now().to_msg(),
        )
        self._msg = build_pointcloud2(points, colors, header)

        # QoS:
        #   rate == 0  → publish once with TRANSIENT_LOCAL so any subscriber
        #                (RViz) that connects *after* we publish still gets
        #                the last sample — the "latched topic" pattern.
        #   rate > 0   → periodic publishing with default volatile QoS.
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
                f'Published once on {topic} '
                f'[{self._msg.width} pts, frame={frame_id}, transient_local]'
            )
        else:
            self._timer = self.create_timer(1.0 / rate_hz, self._publish_cb)
            self.get_logger().info(
                f'Publishing {rate_hz:.1f} Hz on {topic} '
                f'[{self._msg.width} pts, frame={frame_id}]'
            )

    def _publish_cb(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PlyPublisher()
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
