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

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

def load_ply(path, voxel_size):
    """Return (points[N,3] float32, colors[N,3] uint8).

    Open3D reads PLY ``uchar`` RGB as float [0,1]; we scale back to uint8
    because that's what the PointCloud2 RGB packing expects.
    """
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


def pack_rgb_uint32(colors):
    """Pack three uint8 channels into one uint32 field: 0x00RRGGBB.

    ROS PointCloud2 historically squeezes RGB into a single 4-byte slot so a
    point stays at 16 bytes (4 * float32) instead of 19 bytes with padding.
    RViz's 'Color Transformer: RGB8' decodes exactly this layout.
    """
    r = colors[:, 0].astype(np.uint32)
    g = colors[:, 1].astype(np.uint32)
    b = colors[:, 2].astype(np.uint32)
    return (r << 16) | (g << 8) | b


def build_pointcloud2(points, colors, header):
    """Assemble a sensor_msgs/PointCloud2 from XYZ + packed RGB.

    ``dtype_from_fields`` turns the field list into a numpy structured dtype
    whose byte layout matches the PointCloud2 binary format; ``create_cloud``
    then copies that array straight into the message's ``data`` buffer.
    """
    fields = [
        PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32,  count=1),
    ]
    structured = np.empty(len(points), dtype=point_cloud2.dtype_from_fields(fields))
    structured['x'] = points[:, 0]
    structured['y'] = points[:, 1]
    structured['z'] = points[:, 2]
    structured['rgb'] = pack_rgb_uint32(colors)
    return point_cloud2.create_cloud(header, fields, structured)


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
        topic = self.get_parameter('topic').value
        rate_hz = self.get_parameter('publish_rate_hz').value
        voxel = self.get_parameter('voxel_downsample').value

        # PLY is loaded once at startup. Re-reading on every tick would stream
        # the same hundreds of MB over DDS for nothing.
        self.get_logger().info(f'Loading PLY: {ply_path}')
        points, colors = load_ply(ply_path, voxel)
        self.get_logger().info(
            f'Loaded {len(points)} points '
            f'(voxel_downsample={voxel if voxel > 0 else "off"})'
        )

        header = Header()
        header.frame_id = frame_id
        header.stamp = self.get_clock().now().to_msg()
        self._msg = build_pointcloud2(points, colors, header)

        # QoS:
        #   rate == 0  → publish once with TRANSIENT_LOCAL so any subscriber
        #                (RViz) that connects *after* we publish still gets
        #                the last sample. This is the "latched topic" pattern.
        #   rate > 0   → periodic publishing with default QoS.
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
