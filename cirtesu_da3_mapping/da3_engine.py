"""
da3_engine — persistent Depth-Anything-3 wrapper for incremental chunk processing.

Design
------
    Captured frames (on disk) ──► DA3Engine.process_chunk(image_paths) ──► ChunkResult
                                         │
                                         ├─ keeps DA3 model loaded on GPU
                                         ├─ aligns each chunk with the previous one (sim3)
                                         └─ accumulates sim3 so every chunk lands in chunk-0 frame

The engine knows nothing about ROS2, threads or disk layout.  The ROS2 node
feeds it a list of image paths (including the overlap region with the previous
chunk) and receives a ChunkResult with the aligned points ready to publish.
"""

from __future__ import annotations

import json
import os
import sys
from dataclasses import dataclass

import numpy as np
import torch


# ──────────────────────────────────────────────────────────────────────────────
# da3_streaming needs to be importable before we can touch DA3 internals.
# The engine accepts the path in the constructor and prepends it to sys.path.
# ──────────────────────────────────────────────────────────────────────────────

def _prepare_da3_paths(da3_streaming_dir: str, da3_src_dir: str | None) -> None:
    if da3_streaming_dir and da3_streaming_dir not in sys.path:
        sys.path.insert(0, da3_streaming_dir)
    if da3_src_dir and da3_src_dir not in sys.path:
        sys.path.insert(0, da3_src_dir)


# ──────────────────────────────────────────────────────────────────────────────
# Public result type
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class ChunkResult:
    """Everything the publisher needs to display or save one chunk."""
    chunk_idx: int
    points: np.ndarray      # (N, 3) float32 — in chunk-0 frame, confidence-filtered
    colors: np.ndarray      # (N, 3) uint8
    poses_c2w: np.ndarray   # (M, 4, 4) float64 — only the *new* frames in this chunk
    sim3_cum: tuple         # (s, R, t) transform from this chunk's local frame to chunk-0


# ──────────────────────────────────────────────────────────────────────────────
# Engine
# ──────────────────────────────────────────────────────────────────────────────

class DA3Engine:
    """
    Persistent DA3 model + chunk alignment state.

    Usage:
        engine = DA3Engine(config_path, da3_streaming_dir, da3_src_dir)
        for image_paths in chunks:
            result = engine.process_chunk(image_paths)
            publish(result.points, result.colors)
    """

    def __init__(
        self,
        config_path: str,
        da3_streaming_dir: str,
        da3_src_dir: str | None = None,
    ):
        _prepare_da3_paths(da3_streaming_dir, da3_src_dir)

        # Imports deferred until paths are set.
        from loop_utils.config_utils import load_config
        from loop_utils.alignment_torch import (
            apply_sim3_direct_torch,
            depth_to_point_cloud_optimized_torch,
        )
        from loop_utils.sim3utils import (
            accumulate_sim3_transforms,
            precompute_scale_chunks_with_depth,
            weighted_align_point_maps,
        )
        from safetensors.torch import load_file
        from depth_anything_3.api import DepthAnything3

        self._load_config = load_config
        self._apply_sim3 = apply_sim3_direct_torch
        self._depth_to_pc = depth_to_point_cloud_optimized_torch
        self._accumulate_sim3 = accumulate_sim3_transforms
        self._precompute_scale = precompute_scale_chunks_with_depth
        self._align_point_maps = weighted_align_point_maps

        self._cfg = load_config(config_path)
        m = self._cfg["Model"]
        self.chunk_size = int(m["chunk_size"])
        self.overlap = int(m["overlap"])
        self.step = self.chunk_size - self.overlap
        if self.overlap >= self.chunk_size:
            raise ValueError(
                f"overlap ({self.overlap}) must be < chunk_size ({self.chunk_size})"
            )

        self._ref_view_strategy = m["ref_view_strategy"]
        self._process_res = int(m.get("process_res", 504))
        self._process_res_method = m.get("process_res_method", "upper_bound_resize")
        self._align_method = m["align_method"]
        self._scale_method = m.get("scale_compute_method", "auto")
        self._conf_coef = float(m["Pointcloud_Save"]["conf_threshold_coef"])
        self._sample_ratio = float(m["Pointcloud_Save"]["sample_ratio"])

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        if self.device == "cuda" and torch.cuda.get_device_capability()[0] >= 8:
            self.dtype = torch.bfloat16
        else:
            self.dtype = torch.float16

        with open(self._cfg["Weights"]["DA3_CONFIG"]) as f:
            da3_model_cfg = json.load(f)
        self.model = DepthAnything3(**da3_model_cfg)
        self.model.load_state_dict(load_file(self._cfg["Weights"]["DA3"]), strict=False)
        self.model.eval().to(self.device)

        # Chunk history (minimum needed to align the next chunk).
        self._prev_predictions = None
        self._rel_sim3_list: list[tuple] = []   # relative chunk_{i+1} → chunk_i
        self._cum_sim3_list: list[tuple] = []   # cumulative chunk_{i+1} → chunk_0
        self._chunk_idx = 0

    # ── Public API ─────────────────────────────────────────────────────────────

    @property
    def next_chunk_idx(self) -> int:
        return self._chunk_idx

    def reset(self) -> None:
        """Forget all chunk history (the model stays loaded)."""
        self._prev_predictions = None
        self._rel_sim3_list.clear()
        self._cum_sim3_list.clear()
        self._chunk_idx = 0

    def process_chunk(self, image_paths: list[str]) -> ChunkResult:
        """
        Run DA3 on `image_paths`, align with the previous chunk, and return
        the aligned pointcloud plus c2w poses (in the chunk-0 frame).

        Caller contract
        ---------------
            chunk 0  : exactly `chunk_size` fresh images
            chunk k>0: `overlap` last images of chunk k-1, followed by
                       up to `step` new images (total ≤ `chunk_size`)
        """
        chunk_idx = self._chunk_idx
        predictions = self._run_inference(image_paths)

        if chunk_idx == 0:
            sim3_cum = (np.float32(1.0), np.eye(3, dtype=np.float32),
                        np.zeros(3, dtype=np.float32))
        else:
            rel = self._align_with_previous(predictions)
            self._rel_sim3_list.append(rel)
            self._cum_sim3_list = self._accumulate_sim3(self._rel_sim3_list)
            sim3_cum = self._cum_sim3_list[-1]

        points, colors = self._build_pointcloud(predictions, sim3_cum, chunk_idx)
        poses_c2w = self._extract_new_poses(predictions, sim3_cum, chunk_idx)

        # Bookkeeping for the next chunk.
        self._prev_predictions = predictions
        self._chunk_idx += 1
        torch.cuda.empty_cache()

        return ChunkResult(
            chunk_idx=chunk_idx,
            points=points,
            colors=colors,
            poses_c2w=poses_c2w,
            sim3_cum=sim3_cum,
        )

    # ── Inference ──────────────────────────────────────────────────────────────

    def _run_inference(self, image_paths):
        torch.cuda.empty_cache()
        with torch.no_grad(), torch.cuda.amp.autocast(dtype=self.dtype):
            pred = self.model.inference(
                image_paths,
                ref_view_strategy=self._ref_view_strategy,
                process_res=self._process_res,
                process_res_method=self._process_res_method,
            )
            pred.depth = np.squeeze(pred.depth)
            pred.conf -= 1.0
        return pred

    # ── Alignment ──────────────────────────────────────────────────────────────

    def _align_with_previous(self, cur):
        """Estimate sim3 that maps `cur` (current chunk) into the previous chunk's frame."""
        prev = self._prev_predictions
        ovl = self.overlap

        pm_prev = self._depth_to_pc(prev.depth, prev.intrinsics, prev.extrinsics)
        pm_cur = self._depth_to_pc(cur.depth, cur.intrinsics, cur.extrinsics)

        pm1 = _as_numpy(pm_prev)[-ovl:]
        pm2 = _as_numpy(pm_cur)[:ovl]
        conf1 = prev.conf[-ovl:]
        conf2 = cur.conf[:ovl]

        conf_threshold = float(min(np.median(conf1), np.median(conf2))) * 0.1

        precompute_scale = None
        if self._align_method == "scale+se3":
            precompute_scale, *_ = self._precompute_scale(
                np.squeeze(prev.depth[-ovl:]),
                np.squeeze(prev.conf[-ovl:]),
                np.squeeze(cur.depth[:ovl]),
                np.squeeze(cur.conf[:ovl]),
                method=self._scale_method,
            )

        return self._align_point_maps(
            pm1, conf1, pm2, conf2,
            conf_threshold=conf_threshold,
            config=self._cfg,
            precompute_scale=precompute_scale,
        )

    # ── Pointcloud construction ────────────────────────────────────────────────

    def _build_pointcloud(self, pred, sim3_cum, chunk_idx):
        """
        Unproject depth → world points → apply cumulative sim3 → confidence filter.

        For chunk k>0 we skip the first `overlap` frames because they are
        duplicates of the last frames of chunk k-1 (already published).
        """
        world_pm = self._depth_to_pc(pred.depth, pred.intrinsics, pred.extrinsics)
        if chunk_idx > 0:
            s, R, t = sim3_cum
            world_pm = self._apply_sim3(world_pm, s, R, t)
        world_pm = _as_numpy(world_pm)

        start = self.overlap if chunk_idx > 0 else 0
        pts = world_pm[start:].reshape(-1, 3).astype(np.float32)
        clr = pred.processed_images[start:].reshape(-1, 3).astype(np.uint8)
        cfs = pred.conf[start:].reshape(-1).astype(np.float32)

        # Same confidence filter as save_confident_pointcloud_batch in da3_streaming.
        if cfs.size > 0:
            thr = float(np.mean(cfs)) * self._conf_coef
            mask = (cfs >= thr) & (cfs > 1e-5)
            pts = pts[mask]
            clr = clr[mask]
            cfs = cfs[mask]

        if self._sample_ratio < 1.0 and len(pts) > 0:
            n = int(len(pts) * self._sample_ratio)
            idx = np.random.choice(len(pts), n, replace=False)
            pts = pts[idx]
            clr = clr[idx]

        return pts, clr

    # ── Camera poses ───────────────────────────────────────────────────────────

    def _extract_new_poses(self, pred, sim3_cum, chunk_idx):
        """Return c2w 4×4 poses for the *new* frames of this chunk (overlap dropped)."""
        s, R, t = sim3_cum
        S = np.eye(4, dtype=np.float64)
        S[:3, :3] = float(s) * np.asarray(R, dtype=np.float64)
        S[:3, 3] = np.asarray(t, dtype=np.float64)
        s_float = float(s)

        start = self.overlap if chunk_idx > 0 else 0
        poses = []
        for w2c in pred.extrinsics[start:]:
            w2c4 = np.eye(4, dtype=np.float64)
            w2c4[:3, :] = w2c
            c2w = np.linalg.inv(w2c4)
            if chunk_idx > 0:
                c2w = S @ c2w
                c2w[:3, :3] /= s_float
            poses.append(c2w)
        return np.stack(poses) if poses else np.zeros((0, 4, 4), dtype=np.float64)


# ──────────────────────────────────────────────────────────────────────────────
# Small helper
# ──────────────────────────────────────────────────────────────────────────────

def _as_numpy(x):
    return x.cpu().numpy() if torch.is_tensor(x) else x
