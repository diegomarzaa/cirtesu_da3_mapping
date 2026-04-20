"""Session state and chunk-window helpers for DA3 mapping."""

from __future__ import annotations

import os
import threading
from dataclasses import dataclass
from datetime import datetime

import numpy as np

from cirtesu_da3_mapping.da3_engine import ChunkResult
from cirtesu_da3_mapping.pointcloud_utils import save_ply_xyzrgb


@dataclass(frozen=True)
class ChunkWindow:
    start: int
    paths: list[str]
    partial: bool


def iter_chunk_windows(
    frame_paths: list[str],
    chunk_size: int,
    overlap: int,
    start_index: int = 0,
    include_partial: bool = False,
):
    """Yield DA3 chunk windows with the same overlap rule as the live mapper."""
    if chunk_size <= 0:
        raise ValueError(f"chunk_size must be > 0, got {chunk_size}")
    if overlap >= chunk_size:
        raise ValueError(f"overlap ({overlap}) must be < chunk_size ({chunk_size})")

    total = len(frame_paths)
    step = chunk_size - overlap
    start = start_index

    while start < total:
        end = start + chunk_size
        if end <= total:
            yield ChunkWindow(start=start, paths=list(frame_paths[start:end]), partial=False)
            start += step
            continue

        has_new_frames = start == 0 or total > start + overlap
        if include_partial and has_new_frames:
            yield ChunkWindow(start=start, paths=list(frame_paths[start:total]), partial=True)
        break


class MappingSession:
    """Tracks one capture session and the accumulated map."""

    def __init__(self, session_base_dir: str, debug_save: bool):
        self._session_base_dir = session_base_dir
        self._debug_save = debug_save
        self._frames_lock = threading.Lock()
        self._map_lock = threading.Lock()
        self.session_dir: str | None = None
        self.frames_dir: str | None = None
        self.debug_dir: str | None = None
        self._frame_paths: list[str] = []
        self._map_chunks: list[tuple[np.ndarray, np.ndarray]] = []
        self._map_poses: list[np.ndarray] = []

    def start(self) -> str:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(self._session_base_dir, f"session_{ts}")
        self.frames_dir = os.path.join(self.session_dir, "frames")
        os.makedirs(self.frames_dir, exist_ok=True)

        self.debug_dir = None
        if self._debug_save:
            self.debug_dir = os.path.join(self.session_dir, "debug")
            os.makedirs(self.debug_dir, exist_ok=True)

        with self._frames_lock:
            self._frame_paths = []
        with self._map_lock:
            self._map_chunks = []
            self._map_poses = []
        return self.session_dir

    def frame_count(self) -> int:
        with self._frames_lock:
            return len(self._frame_paths)

    def add_frame(self, path: str) -> int:
        with self._frames_lock:
            self._frame_paths.append(path)
            return len(self._frame_paths)

    def frame_paths_snapshot(self) -> list[str]:
        with self._frames_lock:
            return list(self._frame_paths)

    def integrate(self, result: ChunkResult) -> tuple[int, int, int]:
        with self._map_lock:
            if len(result.points) > 0:
                self._map_chunks.append((result.points, result.colors))
            for pose in result.poses_c2w:
                self._map_poses.append(pose)
            return (
                len(self._map_chunks),
                sum(len(points) for points, _ in self._map_chunks),
                len(self._map_poses),
            )

    def stats(self) -> tuple[int, int, int]:
        with self._map_lock:
            return (
                len(self._map_chunks),
                sum(len(points) for points, _ in self._map_chunks),
                len(self._map_poses),
            )

    def accumulated_points_colors(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        with self._map_lock:
            if not self._map_chunks:
                return None, None
            points = np.concatenate([points for points, _ in self._map_chunks], axis=0)
            colors = np.concatenate([colors for _, colors in self._map_chunks], axis=0)
            return points, colors

    def poses_snapshot(self) -> list[np.ndarray]:
        with self._map_lock:
            return list(self._map_poses)

    def debug_dump(self, result: ChunkResult) -> None:
        if self.debug_dir is None:
            return

        ply_path = os.path.join(self.debug_dir, f"chunk_{result.chunk_idx:04d}.ply")
        save_ply_xyzrgb(ply_path, result.points, result.colors)

        meta_path = os.path.join(self.debug_dir, f"chunk_{result.chunk_idx:04d}.npz")
        s, rotation, translation = result.sim3_cum
        np.savez_compressed(
            meta_path,
            sim3_s=np.asarray(s, dtype=np.float64),
            sim3_R=np.asarray(rotation, dtype=np.float64),
            sim3_t=np.asarray(translation, dtype=np.float64),
            poses_c2w=result.poses_c2w,
            num_points=len(result.points),
        )
