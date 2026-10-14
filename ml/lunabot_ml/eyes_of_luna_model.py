"""Compact U-Net with independent categorical-terrain and binary-shadow heads."""
from __future__ import annotations

import math
from pathlib import Path

from .eyes_of_luna_dataset import DEFAULT_TASK_CONFIG, DatasetError, load_task_config


def load_model_config(path: Path = DEFAULT_TASK_CONFIG) -> dict:
    """Load the shared task contract and validate provisional training settings."""
    config = load_task_config(path)
    model = config.get("model", {})
    if model.get("architecture") != "unet_multitask":
        raise DatasetError("model.architecture must be unet_multitask")
    channels = model.get("base_channels")
    if isinstance(channels, bool) or not isinstance(channels, int) or channels < 4:
        raise DatasetError("model.base_channels must be an integer >= 4")
    training = config.get("training", {})
    size = training.get("input_size")
    if (not isinstance(size, list) or len(size) != 2
            or any(isinstance(value, bool) or not isinstance(value, int) or value <= 0
                   for value in size)):
        raise DatasetError("training.input_size must be positive [height,width]")
    height, width = size
    if height % 4 or width % 4:
        raise DatasetError("training.input_size dimensions must be divisible by 4")
    source_width, source_height = config["camera"]["source_resolution"]
    if width * source_height != height * source_width:
        raise DatasetError("training.input_size must preserve the camera aspect ratio")
    for key in ("seed", "batch_size", "epochs", "early_stopping_patience"):
        value = training.get(key)
        minimum = 0 if key == "seed" else 1
        if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
            raise DatasetError(f"training.{key} must be an integer >= {minimum}")
    for key in ("learning_rate", "weight_decay", "terrain_dice_weight",
                "shadow_loss_weight", "shadow_dice_weight"):
        value = training.get(key)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(float(value)) or value < 0
                or (key == "learning_rate" and value == 0)):
            raise DatasetError(f"training.{key} must be finite and non-negative")
    workers = training.get("num_workers")
    if isinstance(workers, bool) or not isinstance(workers, int) or workers < 0:
        raise DatasetError("training.num_workers must be a non-negative integer")
    normalization = config.get("normalization", {})
    for key in ("mean", "std"):
        values = normalization.get(key)
        if (not isinstance(values, list) or len(values) != 3
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(float(value)) for value in values)):
            raise DatasetError(f"normalization.{key} must contain three finite values")
    if any(float(value) <= 0 for value in normalization["std"]):
        raise DatasetError("normalization.std values must be positive")
    threshold = config.get("evaluation", {}).get("shadow_threshold")
    if (isinstance(threshold, bool) or not isinstance(threshold, (int, float))
            or not math.isfinite(float(threshold)) or not 0 < threshold < 1):
        raise DatasetError("evaluation.shadow_threshold must be between 0 and 1")
    return config


def build_model(config: dict):
    """Build the Phase 2 network; returns (terrain_logits, shadow_logits)."""
    try:
        import torch
        from torch import nn
        import torch.nn.functional as F
    except ImportError as exc:
        raise RuntimeError("PyTorch is required for Eyes of Luna training/inference") from exc

    model_config = config.get("model", {})
    if model_config.get("architecture") != "unet_multitask":
        raise ValueError("unsupported Eyes of Luna model architecture")
    channels = model_config.get("base_channels")
    if isinstance(channels, bool) or not isinstance(channels, int) or channels < 4:
        raise ValueError("model.base_channels must be an integer >= 4")

    class ConvBlock(nn.Sequential):
        def __init__(self, incoming: int, outgoing: int):
            super().__init__(
                nn.Conv2d(incoming, outgoing, kernel_size=3, padding=1, bias=False),
                nn.BatchNorm2d(outgoing),
                nn.ReLU(inplace=True),
                nn.Conv2d(outgoing, outgoing, kernel_size=3, padding=1, bias=False),
                nn.BatchNorm2d(outgoing),
                nn.ReLU(inplace=True),
            )

    class EyesOfLunaUNet(nn.Module):
        def __init__(self):
            super().__init__()
            self.encoder1 = ConvBlock(3, channels)
            self.encoder2 = ConvBlock(channels, channels * 2)
            self.bridge = ConvBlock(channels * 2, channels * 4)
            self.decoder2 = ConvBlock(channels * 6, channels * 2)
            self.decoder1 = ConvBlock(channels * 3, channels)
            self.pool = nn.MaxPool2d(2)
            self.terrain_head = nn.Conv2d(channels, 4, kernel_size=1)
            self.shadow_head = nn.Conv2d(channels, 1, kernel_size=1)

        def forward(self, image):
            level1 = self.encoder1(image)
            level2 = self.encoder2(self.pool(level1))
            bridge = self.bridge(self.pool(level2))
            decoded2 = self.decoder2(torch.cat((
                F.interpolate(bridge, size=level2.shape[-2:], mode="bilinear",
                              align_corners=False), level2), dim=1))
            decoded1 = self.decoder1(torch.cat((
                F.interpolate(decoded2, size=level1.shape[-2:], mode="bilinear",
                              align_corners=False), level1), dim=1))
            return self.terrain_head(decoded1), self.shadow_head(decoded1)

    return EyesOfLunaUNet()
