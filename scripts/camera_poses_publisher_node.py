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

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

_DEFAULT_POSES = (
    '/home/diego/Cirtesu/media/da3_outputs/diego_room_few/camera_poses.txt'
)
_DEFAULT_INTRINSICS = (
    '/home/diego/Cirtesu/media/da3_outputs/diego_room_few/intrinsic.txt'
)


def read_camera_poses(path):
    poses = []
    with open(path) as f:
        for line_idx, line in enumerate(f, start=1):
            values = list(map(float, line.strip().split()))
            if not values:
                continue
            if len(values) != 16:
                raise RuntimeError(
                    f'Invalid pose line {line_idx} in {path}: expected 16 values, got {len(values)}'
                )
            poses.append(np.array(values, dtype=np.float64).reshape(4, 4))
    if not poses:
        raise RuntimeError(f'No valid poses found in {path}')
    return poses


def read_intrinsics(path):
    intrinsics = []
    with open(path) as f:
        for line_idx, line in enumerate(f, start=1):
            values = list(map(float, line.strip().split()))
            if not values:
                continue
            if len(values) != 4:
                raise RuntimeError(
                    f'Invalid intrinsics line {line_idx} in {path}: expected 4 values, got {len(values)}'
                )
            fx, fy, cx, cy = values
            intrinsics.append((fx, fy, cx, cy))
    if not intrinsics:
        raise RuntimeError(f'No valid intrinsics found in {path}')
    return intrinsics


def rotation_matrix_to_quaternion_xyzw(rotation):
    """Convert a 3x3 rotation matrix into a normalized quaternion (x, y, z, w)."""
    m = rotation
    trace = np.trace(m)
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    else:
        if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2.0 * np.sqrt(max(1.0 + m[0, 0] - m[1, 1] - m[2, 2], 1e-12))
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * np.sqrt(max(1.0 + m[1, 1] - m[0, 0] - m[2, 2], 1e-12))
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(max(1.0 + m[2, 2] - m[0, 0] - m[1, 1], 1e-12))
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s

    quat = np.array([x, y, z, w], dtype=np.float64)
    quat /= np.linalg.norm(quat)
    return quat


def transform_points(points_cam, c2w):
    """Transform Nx3 camera-frame points to world coordinates using C2W."""
    rot = c2w[:3, :3]
    trans = c2w[:3, 3]
    return (rot @ points_cam.T).T + trans


def xyz_to_point(xyz):
    point = Point()
    point.x = float(xyz[0])
    point.y = float(xyz[1])
    point.z = float(xyz[2])
    return point


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
        self._path_msg = self._build_path(poses, header)
        self._axes_msg = self._build_axes_markers(poses, header, axis_length, axis_line_width)
        self._frustums_msg = self._build_frustum_markers(
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

    def _build_path(self, poses, header):
        path = Path()
        path.header = header
        for idx, c2w in enumerate(poses):
            pose = PoseStamped()
            pose.header = Header(frame_id=header.frame_id, stamp=header.stamp)
            pose.pose.position = xyz_to_point(c2w[:3, 3])
            qx, qy, qz, qw = rotation_matrix_to_quaternion_xyzw(c2w[:3, :3])
            pose.pose.orientation.x = float(qx)
            pose.pose.orientation.y = float(qy)
            pose.pose.orientation.z = float(qz)
            pose.pose.orientation.w = float(qw)
            path.poses.append(pose)
        return path

    def _build_axes_markers(self, poses, header, axis_length, axis_line_width):
        markers = MarkerArray()
        axis_specs = [
            ('x', np.array([1.0, 0.0, 0.0]), (1.0, 0.1, 0.1)),
            ('y', np.array([0.0, 1.0, 0.0]), (0.1, 1.0, 0.1)),
            ('z', np.array([0.0, 0.0, 1.0]), (0.1, 0.4, 1.0)),
        ]

        for marker_id, (_, axis_dir, color) in enumerate(axis_specs):
            marker = Marker()
            marker.header = Header(frame_id=header.frame_id, stamp=header.stamp)
            marker.ns = 'camera_axes'
            marker.id = marker_id
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = axis_line_width
            marker.color.r = float(color[0])
            marker.color.g = float(color[1])
            marker.color.b = float(color[2])
            marker.color.a = 1.0

            for c2w in poses:
                origin = c2w[:3, 3]
                endpoint = origin + c2w[:3, :3] @ (axis_dir * axis_length)
                marker.points.append(xyz_to_point(origin))
                marker.points.append(xyz_to_point(endpoint))

            markers.markers.append(marker)

        return markers

    def _build_frustum_markers(
        self, poses, intrinsics, header, frustum_depth, frustum_line_width
    ):
        marker = Marker()
        marker.header = Header(frame_id=header.frame_id, stamp=header.stamp)
        marker.ns = 'camera_frustums'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = frustum_line_width
        marker.color.r = 1.0
        marker.color.g = 0.75
        marker.color.b = 0.1
        marker.color.a = 1.0

        for c2w, (fx, fy, cx, cy) in zip(poses, intrinsics):
            width = 2.0 * cx
            height = 2.0 * cy
            corners_px = np.array(
                [
                    [0.0, 0.0],
                    [width, 0.0],
                    [width, height],
                    [0.0, height],
                ],
                dtype=np.float64,
            )

            corners_cam = []
            for u, v in corners_px:
                x = ((u - cx) / fx) * frustum_depth
                y = ((v - cy) / fy) * frustum_depth
                z = frustum_depth
                corners_cam.append([x, y, z])
            corners_cam = np.array(corners_cam, dtype=np.float64)
            corners_world = transform_points(corners_cam, c2w)
            origin = c2w[:3, 3]

            for corner in corners_world:
                marker.points.append(xyz_to_point(origin))
                marker.points.append(xyz_to_point(corner))

            edge_pairs = [(0, 1), (1, 2), (2, 3), (3, 0)]
            for a, b in edge_pairs:
                marker.points.append(xyz_to_point(corners_world[a]))
                marker.points.append(xyz_to_point(corners_world[b]))

        return MarkerArray(markers=[marker])


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
