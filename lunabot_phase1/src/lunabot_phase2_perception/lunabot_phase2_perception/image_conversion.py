"""Small, ROS-independent conversion from supported sensor encodings to RGB."""
from __future__ import annotations


class ImageEncodingError(ValueError):
    """Raised when an incoming image cannot be decoded as an RGB frame."""


def decode_image_rgb(image):
    """Decode common 8-bit ROS image encodings while respecting padded rows."""
    try:
        import numpy as np
    except ImportError as exc:
        raise ImageEncodingError("NumPy is required to decode the camera image") from exc

    width, height, step = int(image.width), int(image.height), int(image.step)
    encoding = str(image.encoding).lower()
    channels_by_encoding = {
        "rgb8": (3, "rgb"),
        "r8g8b8": (3, "rgb"),
        "bgr8": (3, "bgr"),
        "b8g8r8": (3, "bgr"),
        "rgba8": (4, "rgba"),
        "bgra8": (4, "bgra"),
        "mono8": (1, "mono"),
    }
    if encoding not in channels_by_encoding:
        raise ImageEncodingError(f"unsupported camera image encoding: {image.encoding}")
    channels, order = channels_by_encoding[encoding]
    minimum_step = width * channels
    if width <= 0 or height <= 0 or step < minimum_step:
        raise ImageEncodingError("camera image has invalid dimensions or row step")
    raw_data = np.frombuffer(image.data, dtype=np.uint8)
    expected_bytes = height * step
    if raw_data.size < expected_bytes:
        raise ImageEncodingError("camera image data is shorter than height * step")
    rows = raw_data[:expected_bytes].reshape(height, step)
    pixels = rows[:, :minimum_step].reshape(height, width, channels)
    if order == "rgb":
        rgb = pixels
    elif order == "bgr":
        rgb = pixels[..., ::-1]
    elif order == "rgba":
        rgb = pixels[..., :3]
    elif order == "bgra":
        rgb = pixels[..., 2::-1]
    else:
        rgb = np.repeat(pixels, 3, axis=2)
    return np.ascontiguousarray(rgb)
