"""Point cloud helpers shared by the new mapping pipeline."""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


POINTCLOUD_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
]


def pack_rgb_uint32(colors: np.ndarray) -> np.ndarray:
    """Pack uint8 RGB colors as 0x00RRGGBB for RViz's RGB8 transformer."""
    r = colors[:, 0].astype(np.uint32)
    g = colors[:, 1].astype(np.uint32)
    b = colors[:, 2].astype(np.uint32)
    return (r << 16) | (g << 8) | b


def build_pointcloud2(
    points: np.ndarray,
    colors: np.ndarray,
    header: Header,
) -> PointCloud2:
    """Build a PointCloud2 with XYZ float32 fields plus packed RGB."""
    structured = np.empty(
        len(points),
        dtype=point_cloud2.dtype_from_fields(POINTCLOUD_FIELDS),
    )
    structured["x"] = points[:, 0]
    structured["y"] = points[:, 1]
    structured["z"] = points[:, 2]
    structured["rgb"] = pack_rgb_uint32(colors)
    return point_cloud2.create_cloud(header, POINTCLOUD_FIELDS, structured)


def voxel_downsample(
    points: np.ndarray,
    colors: np.ndarray,
    voxel_size: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Keep one point per voxel cell using only numpy."""
    if voxel_size <= 0.0 or len(points) == 0:
        return points, colors

    keys = np.floor(points / voxel_size).astype(np.int64)
    keys -= keys.min(axis=0)
    extents = keys.max(axis=0) + 1
    flat = (
        keys[:, 0] * (extents[1] * extents[2])
        + keys[:, 1] * extents[2]
        + keys[:, 2]
    )
    _, idx = np.unique(flat, return_index=True)
    return points[idx], colors[idx]


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


def load_ply(path: str, voxel_size: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Load a PLY as float32 XYZ and uint8 RGB."""
    try:
        return _load_ply_open3d(path, voxel_size)
    except ImportError:
        return _load_ply_plyfile(path, voxel_size)


def _load_ply_open3d(path: str, voxel_size: float) -> tuple[np.ndarray, np.ndarray]:
    import open3d as o3d

    pcd = o3d.io.read_point_cloud(path)
    if len(pcd.points) == 0:
        raise RuntimeError(f"PLY is empty or unreadable: {path}")
    if voxel_size > 0.0:
        pcd = pcd.voxel_down_sample(voxel_size)

    points = np.asarray(pcd.points, dtype=np.float32)
    if pcd.has_colors():
        colors = (np.asarray(pcd.colors) * 255.0).astype(np.uint8)
    else:
        colors = np.full((len(points), 3), 255, dtype=np.uint8)
    return points, colors


def _load_ply_plyfile(path: str, voxel_size: float) -> tuple[np.ndarray, np.ndarray]:
    from plyfile import PlyData

    ply = PlyData.read(path)
    if "vertex" not in ply:
        raise RuntimeError(f"PLY has no vertex element: {path}")

    vertex = ply["vertex"].data
    names = vertex.dtype.names or ()
    for field in ("x", "y", "z"):
        if field not in names:
            raise RuntimeError(f"PLY vertex element has no {field!r} field: {path}")

    points = np.column_stack((vertex["x"], vertex["y"], vertex["z"])).astype(np.float32)
    if len(points) == 0:
        raise RuntimeError(f"PLY is empty or unreadable: {path}")

    if all(field in names for field in ("red", "green", "blue")):
        colors = np.column_stack(
            (vertex["red"], vertex["green"], vertex["blue"])
        ).astype(np.uint8)
    elif all(field in names for field in ("r", "g", "b")):
        colors = np.column_stack((vertex["r"], vertex["g"], vertex["b"])).astype(np.uint8)
    else:
        colors = np.full((len(points), 3), 255, dtype=np.uint8)

    points, colors = voxel_downsample(points, colors, voxel_size)
    return points, colors


def save_ply_xyzrgb(path: str, points: np.ndarray, colors: np.ndarray) -> None:
    """Write a binary little-endian XYZRGB PLY for debug outputs."""
    with open(path, "wb") as f:
        header = (
            "ply\nformat binary_little_endian 1.0\n"
            f"element vertex {len(points)}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            "end_header\n"
        ).encode()
        f.write(header)

        rows = np.empty(
            len(points),
            dtype=[
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("r", "u1"),
                ("g", "u1"),
                ("b", "u1"),
            ],
        )
        rows["x"] = points[:, 0]
        rows["y"] = points[:, 1]
        rows["z"] = points[:, 2]
        rows["r"] = colors[:, 0]
        rows["g"] = colors[:, 1]
        rows["b"] = colors[:, 2]
        f.write(rows.tobytes())
