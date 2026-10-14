"""PyTorch loader for Phase 2 RGB frames and independent reviewed masks."""
from __future__ import annotations

from pathlib import Path

from .eyes_of_luna_dataset import DatasetError, load_capture_group_split, load_manifest
from .pillow_compat import resampling_filter


def create_dataset(root: Path, split: str, config: dict, training: bool = False):
    try:
        import numpy as np
        import torch
        from PIL import Image, ImageEnhance, ImageOps
        from torch.utils.data import Dataset
    except ImportError as exc:
        raise RuntimeError(
            "PyTorch, NumPy, and Pillow are required; install ml/requirements.txt") from exc

    root = Path(root)
    if split not in ("train", "validation", "test"):
        raise ValueError(f"unknown split: {split}")
    split_data = load_capture_group_split(root)
    manifest = load_manifest(root)
    selected = set(split_data["splits"][split]["samples"])
    records = [item for item in manifest["samples"] if item["sample_id"] in selected]
    if not records or len(records) != len(selected):
        raise DatasetError(f"{split} split is empty or references missing samples")

    height, width = config["training"]["input_size"]
    size = (width, height)
    mean = np.asarray(config["normalization"]["mean"], dtype=np.float32)
    std = np.asarray(config["normalization"]["std"], dtype=np.float32)

    class EyesOfLunaDataset(Dataset):
        def __len__(self):
            return len(records)

        def __getitem__(self, index):
            record = records[index]
            image = Image.open(root / record["paths"]["rgb"]).convert("RGB")
            terrain = Image.open(root / record["paths"]["terrain_mask"]).convert("L")
            shadow = Image.open(root / record["paths"]["shadow_mask"]).convert("L")

            if training:
                # Geometry is shared by RGB and both masks. Photometric changes
                # affect RGB only; no synthetic shadow is added without a label.
                if bool(torch.rand(()) < 0.5):
                    image, terrain, shadow = (ImageOps.mirror(value)
                                              for value in (image, terrain, shadow))
                brightness = 0.8 + 0.4 * float(torch.rand(()))
                contrast = 0.8 + 0.4 * float(torch.rand(()))
                image = ImageEnhance.Brightness(image).enhance(brightness)
                image = ImageEnhance.Contrast(image).enhance(contrast)

            image = image.resize(size, resampling_filter(Image, "BILINEAR"))
            terrain = terrain.resize(size, resampling_filter(Image, "NEAREST"))
            shadow = shadow.resize(size, resampling_filter(Image, "NEAREST"))
            array = np.asarray(image, dtype=np.float32) / 255.0
            array = ((array - mean) / std).transpose(2, 0, 1).copy()
            terrain_array = np.asarray(terrain, dtype=np.int64).copy()
            shadow_array = np.asarray(shadow, dtype=np.int64).copy()
            return (
                torch.from_numpy(array.astype(np.float32, copy=False)),
                torch.from_numpy(terrain_array),
                torch.from_numpy(shadow_array),
                record["sample_id"],
            )

    return EyesOfLunaDataset()
