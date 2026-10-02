"""Manifest-backed PyTorch dataset with split/world leakage enforcement."""
from __future__ import annotations
import json
from pathlib import Path
from .augmentations import augment
from .dataset_manifest import load_manifest


def create_dataset(root: Path, split: str, config: dict, training=False):
    try:
        import numpy as np
        import torch
        from PIL import Image
        from torch.utils.data import Dataset
    except ImportError as exc:
        raise RuntimeError("Install ml/requirements.txt to load training data") from exc
    root = Path(root)
    manifest = load_manifest(root)
    split_data = json.loads((root / "splits/world_split.json").read_text(encoding="utf-8"))
    if split not in split_data["splits"]:
        raise ValueError(f"unknown split: {split}")
    selected = set(split_data["splits"][split]["samples"])
    records = [item for item in manifest["samples"] if item["sample_id"] in selected]
    if len(records) != len(selected):
        raise ValueError("split references missing samples")
    mean, std = config["normalization"]["mean"], config["normalization"]["std"]
    size = tuple(config["data"]["image_size"])[::-1]
    seed = int(config["training"]["seed"])

    class TerrainDataset(Dataset):
        def __len__(self): return len(records)
        def __getitem__(self, index):
            record = records[index]
            image = Image.open(root / record["paths"]["rgb"]).convert("RGB")
            mask = Image.open(root / record["paths"]["mask"]).convert("L")
            image, mask = augment(image, mask, seed + index, training)
            image = image.resize(size, Image.Resampling.BILINEAR)
            mask = mask.resize(size, Image.Resampling.NEAREST)
            array = np.asarray(image, dtype=np.float32).transpose(2, 0, 1) / 255.0
            array = (array - np.asarray(mean)[:, None, None]) / np.asarray(std)[:, None, None]
            labels = np.asarray(mask, dtype=np.int64)
            return torch.from_numpy(array.astype(np.float32)), torch.from_numpy(labels.copy()), record["sample_id"]
    return TerrainDataset()
