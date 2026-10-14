"""Pillow resampling enum compatibility for older Pillow 9.x releases."""
from __future__ import annotations


def resampling_filter(image_module, name: str):
    """Resolve a Pillow resampling filter before/after the 9.1 enum change."""
    namespace = getattr(image_module, "Resampling", image_module)
    try:
        return getattr(namespace, name)
    except AttributeError as exc:
        raise ValueError(f"Pillow does not provide resampling filter {name!r}") from exc
