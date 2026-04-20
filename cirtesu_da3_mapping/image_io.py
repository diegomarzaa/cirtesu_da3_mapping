"""Image topic detection and ROS image saving helpers."""

from __future__ import annotations

import os
from typing import Literal

import numpy as np
from sensor_msgs.msg import CompressedImage, Image

RAW_IMAGE_TYPE = "sensor_msgs/msg/Image"
COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"
ImageTopicKind = Literal["raw", "compressed"]


def resolve_image_topic_kind(topic_names_and_types, image_topic: str) -> ImageTopicKind | None:
    """Return the supported image type currently advertised on a topic."""
    for topic_name, topic_types in topic_names_and_types:
        if topic_name != image_topic:
            continue
        if COMPRESSED_IMAGE_TYPE in topic_types:
            return "compressed"
        if RAW_IMAGE_TYPE in topic_types:
            return "raw"
    return None


def message_stamp_ns(msg, fallback_ns: int) -> int:
    stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
    return stamp_ns if stamp_ns > 0 else fallback_ns


def save_image_message(msg, frames_dir: str, index: int) -> str:
    """Save sensor_msgs/Image or sensor_msgs/CompressedImage as a PNG."""
    path = os.path.join(frames_dir, f"{index:06d}.png")
    if isinstance(msg, CompressedImage):
        _save_compressed_image(msg, path)
    elif isinstance(msg, Image):
        _save_raw_image(msg, path)
    else:
        raise TypeError(f"Unsupported image message type: {type(msg).__name__}")
    return path


def _save_compressed_image(msg: CompressedImage, path: str) -> None:
    import cv2

    cv_img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    if cv_img is None:
        raise ValueError("cv2.imdecode returned None for compressed image")
    if not cv2.imwrite(path, cv_img):
        raise RuntimeError(f"Failed to write image: {path}")


def _save_raw_image(msg: Image, path: str) -> None:
    try:
        from cv_bridge import CvBridge
        import cv2

        cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if not cv2.imwrite(path, cv_img):
            raise RuntimeError(f"Failed to write image: {path}")
        return
    except Exception:
        pass

    enc = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)
    height, width = msg.height, msg.width

    if enc in ("rgb8", "bgr8"):
        img = data.reshape(height, width, 3)
        if enc == "rgb8":
            img = img[:, :, ::-1]
    elif enc == "mono8":
        img = data.reshape(height, width)
    elif enc in ("rgba8", "bgra8"):
        img = data.reshape(height, width, 4)[:, :, :3]
        if enc == "rgba8":
            img = img[:, :, ::-1]
    else:
        raise ValueError(f"Unsupported image encoding: {enc}")

    try:
        import cv2

        if not cv2.imwrite(path, img):
            raise RuntimeError(f"Failed to write image: {path}")
    except ImportError:
        from PIL import Image as PILImage

        if enc != "mono8":
            img = img[:, :, ::-1]
        mode = "L" if enc == "mono8" else "RGB"
        PILImage.fromarray(img, mode).save(path)
