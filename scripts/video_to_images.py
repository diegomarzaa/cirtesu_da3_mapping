#!/usr/bin/env python3
"""Extract sampled frames from a video into an image folder."""

from __future__ import annotations

import argparse
import os
import sys

import cv2


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract frames from a video for DA3 folder processing."
    )
    parser.add_argument("video_path", help="Input video path.")
    parser.add_argument("output_dir", help="Directory where extracted images are written.")
    parser.add_argument(
        "--fps",
        type=float,
        default=5.0,
        help="Sampling FPS. Use <= 0 to keep every frame. Default: 5.0.",
    )
    parser.add_argument(
        "--every-n",
        type=int,
        default=0,
        help="Save every Nth frame. Overrides --fps when > 0.",
    )
    parser.add_argument(
        "--start-sec",
        type=float,
        default=0.0,
        help="Start time in seconds. Default: 0.",
    )
    parser.add_argument(
        "--end-sec",
        type=float,
        default=0.0,
        help="End time in seconds. Use 0 to read until EOF. Default: 0.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Maximum number of frames to write. Use 0 for no limit.",
    )
    parser.add_argument(
        "--max-width",
        type=int,
        default=0,
        help="Downscale frames to this max width while preserving aspect ratio.",
    )
    parser.add_argument(
        "--prefix",
        default="frame",
        help="Output filename prefix. Default: frame.",
    )
    parser.add_argument(
        "--ext",
        choices=("png", "jpg", "jpeg"),
        default="png",
        help="Output image extension. Default: png.",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
        help="JPEG quality when --ext is jpg/jpeg. Default: 95.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Allow writing into a non-empty output directory.",
    )
    return parser.parse_args()


def _frame_interval(video_fps: float, sample_fps: float, every_n: int) -> int:
    if every_n > 0:
        return every_n
    if sample_fps <= 0.0 or video_fps <= 0.0:
        return 1
    return max(1, round(video_fps / sample_fps))


def _resize_if_needed(frame, max_width: int):
    if max_width <= 0:
        return frame
    height, width = frame.shape[:2]
    if width <= max_width:
        return frame
    scale = max_width / float(width)
    target_size = (max_width, max(1, round(height * scale)))
    return cv2.resize(frame, target_size, interpolation=cv2.INTER_AREA)


def _imwrite_params(ext: str, jpeg_quality: int) -> list[int]:
    if ext in ("jpg", "jpeg"):
        quality = min(100, max(1, jpeg_quality))
        return [cv2.IMWRITE_JPEG_QUALITY, quality]
    return []


def main() -> int:
    args = _parse_args()
    if not os.path.isfile(args.video_path):
        print(f"[ERROR] Video does not exist: {args.video_path}", file=sys.stderr)
        return 2

    os.makedirs(args.output_dir, exist_ok=True)
    if os.listdir(args.output_dir) and not args.overwrite:
        print(
            f"[ERROR] Output directory is not empty: {args.output_dir}\n"
            "        Pass --overwrite if you want to append/replace matching names.",
            file=sys.stderr,
        )
        return 2

    cap = cv2.VideoCapture(args.video_path)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open video: {args.video_path}", file=sys.stderr)
        return 1

    video_fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    interval = _frame_interval(video_fps, args.fps, args.every_n)
    actual_fps = video_fps / interval if video_fps > 0.0 else 0.0
    start_frame = max(0, round(args.start_sec * video_fps)) if video_fps > 0.0 else 0
    end_frame = round(args.end_sec * video_fps) if args.end_sec > 0.0 and video_fps > 0.0 else 0
    params = _imwrite_params(args.ext, args.jpeg_quality)

    if start_frame > 0:
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    read_count = start_frame
    saved_count = 0
    while True:
        if end_frame > 0 and read_count >= end_frame:
            break
        ok, frame = cap.read()
        if not ok:
            break

        if (read_count - start_frame) % interval == 0:
            frame = _resize_if_needed(frame, args.max_width)
            output_path = os.path.join(
                args.output_dir,
                f"{args.prefix}_{saved_count:06d}.{args.ext}",
            )
            if not cv2.imwrite(output_path, frame, params):
                cap.release()
                print(f"[ERROR] Could not write image: {output_path}", file=sys.stderr)
                return 1
            saved_count += 1
            if args.max_frames > 0 and saved_count >= args.max_frames:
                break

        read_count += 1

    cap.release()
    print(
        f"Video: {args.video_path}\n"
        f"Video FPS: {video_fps:.3f}, total frames: {total_frames}\n"
        f"Sampling: every {interval} frame(s), approx {actual_fps:.3f} FPS\n"
        f"Saved frames: {saved_count}\n"
        f"Output dir: {args.output_dir}"
    )
    return 0 if saved_count > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
