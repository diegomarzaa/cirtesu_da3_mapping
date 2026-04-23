"""Shared DA3 batch reconstruction helpers."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from cirtesu_da3_mapping.pointcloud_utils import load_da3_glb, load_ply


IMAGE_EXTENSIONS = ("png", "jpg", "jpeg")


@dataclass(frozen=True)
class BatchReconstructionConfig:
    da3_root_dir: str
    da3_cli: str
    da3_model_dir: str
    da3_streaming_dir: str
    streaming_config: str
    max_normal_images: int = 80
    process_res: int = 504
    num_max_points: int = 4_000_000
    conf_thresh_percentile: float = 10.0
    show_cameras: bool = False
    fallback_to_streaming: bool = True
    voxel_downsample: float = 0.0
    command_timeout_sec: float = 0.0
    save_depth_outputs: bool = True


@dataclass(frozen=True)
class BatchReconstructionResult:
    mode: str
    image_count: int
    output_path: str
    output_dir: str
    depth_dir: str
    points: np.ndarray
    colors: np.ndarray


LogFn = Callable[[str], None]


class Da3BatchReconstructor:
    """Run DA3 normal first, falling back to DA3-Streaming when needed."""

    def __init__(
        self,
        config: BatchReconstructionConfig,
        log_info: LogFn | None = None,
        log_warning: LogFn | None = None,
    ):
        self.config = config
        self._log_info = log_info or (lambda _msg: None)
        self._log_warning = log_warning or self._log_info
        self.validate_config()

    def validate_config(self) -> None:
        cfg = self.config
        missing = []
        if not cfg.da3_root_dir or not os.path.isdir(cfg.da3_root_dir):
            missing.append(f"da3_root_dir does not exist: {cfg.da3_root_dir}")
        if not cfg.da3_model_dir or not os.path.isdir(cfg.da3_model_dir):
            missing.append(f"da3_model_dir does not exist: {cfg.da3_model_dir}")
        if not cfg.da3_streaming_dir or not os.path.isdir(cfg.da3_streaming_dir):
            missing.append(f"da3_streaming_dir does not exist: {cfg.da3_streaming_dir}")
        if not cfg.streaming_config or not os.path.isfile(cfg.streaming_config):
            missing.append(f"streaming_config does not exist: {cfg.streaming_config}")
        if shutil.which(cfg.da3_cli) is None and not os.path.isfile(cfg.da3_cli):
            missing.append(f"da3_cli is not executable or in PATH: {cfg.da3_cli}")
        if cfg.max_normal_images < 1:
            missing.append("max_normal_images must be >= 1")
        if cfg.process_res < 1:
            missing.append("process_res must be >= 1")
        if cfg.num_max_points < 1:
            missing.append("num_max_points must be >= 1")
        if missing:
            raise RuntimeError("Invalid DA3 batch configuration:\n  - " + "\n  - ".join(missing))

    def reconstruct(
        self,
        input_dir: str,
        output_dir: str,
        force_streaming: bool = False,
    ) -> BatchReconstructionResult:
        image_count = len(find_image_files(input_dir))
        if image_count < 1:
            raise RuntimeError(f"No images found in: {input_dir}")

        os.makedirs(output_dir, exist_ok=True)
        use_streaming = force_streaming or image_count > self.config.max_normal_images

        if not use_streaming:
            try:
                return self._run_da3_normal(
                    input_dir,
                    os.path.join(output_dir, "da3_normal"),
                    image_count,
                )
            except RuntimeError as exc:
                if (
                    not self.config.fallback_to_streaming
                    or not looks_like_recoverable_normal_failure(str(exc))
                ):
                    raise
                self._log_warning(
                    "DA3 normal failed; falling back to DA3-Streaming."
                )
                use_streaming = True

        if use_streaming:
            return self._run_da3_streaming(
                input_dir,
                os.path.join(output_dir, "da3_streaming"),
                image_count,
            )

        raise RuntimeError("Unexpected DA3 reconstruction state.")

    def _run_da3_normal(
        self,
        input_dir: str,
        output_dir: str,
        image_count: int,
    ) -> BatchReconstructionResult:
        cfg = self.config
        cmd = [
            cfg.da3_cli,
            "images",
            input_dir,
            "--model-dir",
            cfg.da3_model_dir,
            "--export-dir",
            output_dir,
            "--export-format",
            "mini_npz-glb" if cfg.save_depth_outputs else "glb",
            "--process-res",
            str(cfg.process_res),
            "--num-max-points",
            str(cfg.num_max_points),
            "--conf-thresh-percentile",
            str(cfg.conf_thresh_percentile),
            "--auto-cleanup",
        ]
        cmd.append("--show-cameras" if cfg.show_cameras else "--no-show-cameras")
        self._run_command(cmd, cwd=cfg.da3_root_dir, label="DA3 normal")

        glb_path = os.path.join(output_dir, "scene.glb")
        if not os.path.isfile(glb_path):
            raise RuntimeError(f"DA3 normal did not produce scene.glb: {glb_path}")

        points, colors = load_da3_glb(glb_path, cfg.voxel_downsample)
        depth_dir = (
            save_normal_depth_outputs(output_dir)
            if cfg.save_depth_outputs
            else ""
        )
        return BatchReconstructionResult(
            mode="da3_normal",
            image_count=image_count,
            output_path=glb_path,
            output_dir=output_dir,
            depth_dir=depth_dir,
            points=points,
            colors=colors,
        )

    def _run_da3_streaming(
        self,
        input_dir: str,
        output_dir: str,
        image_count: int,
    ) -> BatchReconstructionResult:
        cfg = self.config
        cmd = [
            sys.executable,
            "da3_streaming.py",
            "--image_dir",
            input_dir,
            "--config",
            cfg.streaming_config,
            "--output_dir",
            output_dir,
        ]
        self._run_command(cmd, cwd=cfg.da3_streaming_dir, label="DA3-Streaming")

        ply_path = os.path.join(output_dir, "pcd", "combined_pcd.ply")
        if not os.path.isfile(ply_path):
            raise RuntimeError(f"DA3-Streaming did not produce combined_pcd.ply: {ply_path}")

        points, colors = load_ply(ply_path, cfg.voxel_downsample)
        depth_dir = (
            save_streaming_depth_outputs(output_dir)
            if cfg.save_depth_outputs
            else ""
        )
        return BatchReconstructionResult(
            mode="da3_streaming",
            image_count=image_count,
            output_path=ply_path,
            output_dir=output_dir,
            depth_dir=depth_dir,
            points=points,
            colors=colors,
        )

    def _run_command(self, cmd: list[str], cwd: str, label: str) -> None:
        self._log_info(f"Running {label}: {' '.join(cmd)}")
        try:
            result = subprocess.run(
                cmd,
                cwd=cwd,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=(
                    self.config.command_timeout_sec
                    if self.config.command_timeout_sec > 0
                    else None
                ),
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"{label} timed out after {exc.timeout} seconds") from exc

        output = result.stdout or ""
        if result.returncode != 0:
            tail = "\n".join(output.splitlines()[-80:])
            raise RuntimeError(
                f"{label} failed with return code {result.returncode}.\n{tail}"
            )
        if output:
            tail = "\n".join(output.splitlines()[-12:])
            self._log_info(f"{label} finished.\n{tail}")


def find_image_files(
    image_dir: str,
    extensions: tuple[str, ...] = IMAGE_EXTENSIONS,
) -> list[str]:
    allowed = {ext.lower().lstrip(".") for ext in extensions}
    paths = []
    for path in Path(image_dir).iterdir():
        if path.is_file() and path.suffix.lower().lstrip(".") in allowed:
            paths.append(str(path))
    return sorted(paths)


def prepare_input_dir(frame_paths: list[str], input_dir: str) -> None:
    os.makedirs(input_dir, exist_ok=True)
    for idx, src in enumerate(frame_paths):
        suffix = Path(src).suffix.lower() or ".png"
        dst = os.path.join(input_dir, f"{idx:06d}{suffix}")
        if os.path.exists(dst):
            continue
        try:
            os.symlink(src, dst)
        except OSError:
            shutil.copy2(src, dst)


def save_normal_depth_outputs(output_dir: str) -> str:
    depth_dir = os.path.join(output_dir, "depth")
    npz_path = os.path.join(output_dir, "exports", "mini_npz", "results.npz")
    if os.path.isfile(npz_path):
        _copy_file(npz_path, os.path.join(depth_dir, "npz", "results.npz"))

    depth_vis_dir = os.path.join(output_dir, "depth_vis")
    if os.path.isdir(depth_vis_dir):
        _copy_tree_files(depth_vis_dir, os.path.join(depth_dir, "colored"))
    return depth_dir


def save_streaming_depth_outputs(output_dir: str) -> str:
    depth_dir = os.path.join(output_dir, "depth")
    source_dir = os.path.join(output_dir, "results_output")
    if not os.path.isdir(source_dir):
        return depth_dir

    npz_dir = os.path.join(depth_dir, "npz")
    colored_dir = os.path.join(depth_dir, "colored")
    os.makedirs(npz_dir, exist_ok=True)
    os.makedirs(colored_dir, exist_ok=True)

    for src in sorted(Path(source_dir).glob("*.npz")):
        dst_npz = os.path.join(npz_dir, src.name)
        if not os.path.exists(dst_npz):
            shutil.copy2(src, dst_npz)
        _save_streaming_depth_preview(str(src), colored_dir)
    return depth_dir


def _copy_file(src: str, dst: str) -> None:
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy2(src, dst)


def _copy_tree_files(src_dir: str, dst_dir: str) -> None:
    os.makedirs(dst_dir, exist_ok=True)
    for name in sorted(os.listdir(src_dir)):
        src = os.path.join(src_dir, name)
        if os.path.isfile(src):
            shutil.copy2(src, os.path.join(dst_dir, name))


def _save_streaming_depth_preview(npz_path: str, output_dir: str) -> None:
    import cv2

    data = np.load(npz_path)
    if "depth" not in data:
        return

    depth = np.asarray(data["depth"], dtype=np.float32)
    finite = np.isfinite(depth)
    if not np.any(finite):
        return

    valid_depth = depth[finite]
    lo = float(np.percentile(valid_depth, 2.0))
    hi = float(np.percentile(valid_depth, 98.0))
    if hi <= lo:
        hi = lo + 1.0

    depth_norm = np.clip((depth - lo) / (hi - lo), 0.0, 1.0)
    depth_u8 = (depth_norm * 255.0).astype(np.uint8)
    depth_bgr = cv2.applyColorMap(depth_u8, cv2.COLORMAP_VIRIDIS)

    if "image" in data:
        image = np.asarray(data["image"], dtype=np.uint8)
        image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if image_bgr.shape[:2] != depth_bgr.shape[:2]:
            image_bgr = cv2.resize(
                image_bgr,
                (depth_bgr.shape[1], depth_bgr.shape[0]),
                interpolation=cv2.INTER_AREA,
            )
        preview = np.concatenate([image_bgr, depth_bgr], axis=1)
    else:
        preview = depth_bgr

    name = Path(npz_path).with_suffix(".jpg").name
    cv2.imwrite(os.path.join(output_dir, name), preview)


def looks_like_recoverable_normal_failure(text: str) -> bool:
    lowered = text.lower()
    return (
        "outofmemoryerror" in lowered
        or "cuda out of memory" in lowered
        or "can't export empty scenes" in lowered
        or "can't export empty scene" in lowered
        or "empty scenes" in lowered
        or "glb has no pointcloud geometry" in lowered
    )
