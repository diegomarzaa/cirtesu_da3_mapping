"""Camera pose and marker helpers for DA3 outputs."""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def read_camera_poses(path: str) -> list[np.ndarray]:
    """Read one flattened 4x4 C2W matrix per line."""
    poses = []
    with open(path) as f:
        for line_idx, line in enumerate(f, start=1):
            values = list(map(float, line.strip().split()))
            if not values:
                continue
            if len(values) != 16:
                raise RuntimeError(
                    f"Invalid pose line {line_idx} in {path}: "
                    f"expected 16 values, got {len(values)}"
                )
            poses.append(np.array(values, dtype=np.float64).reshape(4, 4))
    if not poses:
        raise RuntimeError(f"No valid poses found in {path}")
    return poses


def read_intrinsics(path: str) -> list[tuple[float, float, float, float]]:
    """Read one fx fy cx cy tuple per line."""
    intrinsics = []
    with open(path) as f:
        for line_idx, line in enumerate(f, start=1):
            values = list(map(float, line.strip().split()))
            if not values:
                continue
            if len(values) != 4:
                raise RuntimeError(
                    f"Invalid intrinsics line {line_idx} in {path}: "
                    f"expected 4 values, got {len(values)}"
                )
            intrinsics.append(tuple(values))
    if not intrinsics:
        raise RuntimeError(f"No valid intrinsics found in {path}")
    return intrinsics


def rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a normalized x,y,z,w quaternion."""
    m = rotation
    trace = np.trace(m)
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        quat = np.array([
            (m[2, 1] - m[1, 2]) * s,
            (m[0, 2] - m[2, 0]) * s,
            (m[1, 0] - m[0, 1]) * s,
            0.25 / s,
        ])
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(max(1.0 + m[0, 0] - m[1, 1] - m[2, 2], 1e-12))
        quat = np.array([
            0.25 * s,
            (m[0, 1] + m[1, 0]) / s,
            (m[0, 2] + m[2, 0]) / s,
            (m[2, 1] - m[1, 2]) / s,
        ])
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(max(1.0 + m[1, 1] - m[0, 0] - m[2, 2], 1e-12))
        quat = np.array([
            (m[0, 1] + m[1, 0]) / s,
            0.25 * s,
            (m[1, 2] + m[2, 1]) / s,
            (m[0, 2] - m[2, 0]) / s,
        ])
    else:
        s = 2.0 * np.sqrt(max(1.0 + m[2, 2] - m[0, 0] - m[1, 1], 1e-12))
        quat = np.array([
            (m[0, 2] + m[2, 0]) / s,
            (m[1, 2] + m[2, 1]) / s,
            0.25 * s,
            (m[1, 0] - m[0, 1]) / s,
        ])

    quat /= np.linalg.norm(quat)
    return tuple(float(v) for v in quat)


def xyz_to_point(xyz: np.ndarray) -> Point:
    point = Point()
    point.x = float(xyz[0])
    point.y = float(xyz[1])
    point.z = float(xyz[2])
    return point


def build_path(poses_c2w: list[np.ndarray], header: Header) -> Path:
    path = Path(header=header)
    for c2w in poses_c2w:
        pose = PoseStamped(header=header)
        pose.pose.position = xyz_to_point(c2w[:3, 3])
        qx, qy, qz, qw = rotation_matrix_to_quaternion_xyzw(c2w[:3, :3])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        path.poses.append(pose)
    return path


def build_axes_markers(
    poses_c2w: list[np.ndarray],
    header: Header,
    axis_length: float,
    axis_line_width: float,
) -> MarkerArray:
    markers = MarkerArray()
    axis_specs = [
        (np.array([1.0, 0.0, 0.0]), (1.0, 0.1, 0.1)),
        (np.array([0.0, 1.0, 0.0]), (0.1, 1.0, 0.1)),
        (np.array([0.0, 0.0, 1.0]), (0.1, 0.4, 1.0)),
    ]

    for marker_id, (axis_dir, color) in enumerate(axis_specs):
        marker = Marker()
        marker.header = Header(frame_id=header.frame_id, stamp=header.stamp)
        marker.ns = "camera_axes"
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = axis_line_width
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        for c2w in poses_c2w:
            origin = c2w[:3, 3]
            endpoint = origin + c2w[:3, :3] @ (axis_dir * axis_length)
            marker.points.append(xyz_to_point(origin))
            marker.points.append(xyz_to_point(endpoint))
        markers.markers.append(marker)

    return markers


def build_frustum_markers(
    poses_c2w: list[np.ndarray],
    intrinsics: list[tuple[float, float, float, float]],
    header: Header,
    frustum_depth: float,
    frustum_line_width: float,
) -> MarkerArray:
    marker = Marker()
    marker.header = Header(frame_id=header.frame_id, stamp=header.stamp)
    marker.ns = "camera_frustums"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = frustum_line_width
    marker.color.r = 1.0
    marker.color.g = 0.75
    marker.color.b = 0.1
    marker.color.a = 1.0

    for c2w, (fx, fy, cx, cy) in zip(poses_c2w, intrinsics):
        width = 2.0 * cx
        height = 2.0 * cy
        corners_cam = np.array(
            [
                [((0.0 - cx) / fx) * frustum_depth, ((0.0 - cy) / fy) * frustum_depth, frustum_depth],
                [((width - cx) / fx) * frustum_depth, ((0.0 - cy) / fy) * frustum_depth, frustum_depth],
                [((width - cx) / fx) * frustum_depth, ((height - cy) / fy) * frustum_depth, frustum_depth],
                [((0.0 - cx) / fx) * frustum_depth, ((height - cy) / fy) * frustum_depth, frustum_depth],
            ],
            dtype=np.float64,
        )
        corners_world = (c2w[:3, :3] @ corners_cam.T).T + c2w[:3, 3]
        origin = c2w[:3, 3]

        for corner in corners_world:
            marker.points.append(xyz_to_point(origin))
            marker.points.append(xyz_to_point(corner))
        for a, b in [(0, 1), (1, 2), (2, 3), (3, 0)]:
            marker.points.append(xyz_to_point(corners_world[a]))
            marker.points.append(xyz_to_point(corners_world[b]))

    return MarkerArray(markers=[marker])
