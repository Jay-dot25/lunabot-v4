"""Checkpoint loading and ROS-independent Phase 2 RGB inference."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from .eyes_of_luna_model import build_model, load_model_config


class EyesOfLunaInferenceError(RuntimeError):
    pass


def config_sha256(config: dict) -> str:
    canonical = json.dumps(config, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _resolve_device(torch, device: str) -> str:
    device = str(device).lower()
    if device not in ("cpu", "cuda", "auto"):
        raise EyesOfLunaInferenceError("device must be cpu, cuda, or auto")
    if device == "cuda" and not torch.cuda.is_available():
        raise EyesOfLunaInferenceError("CUDA requested but unavailable")
    return ("cuda" if device == "auto" and torch.cuda.is_available()
            else "cpu" if device == "auto" else device)


def load_checkpoint(model_path: Path, config: dict, device: str = "cpu"):
    try:
        import torch
    except ImportError as exc:
        raise EyesOfLunaInferenceError("PyTorch is required for inference") from exc
    model_path = Path(model_path)
    if not model_path.is_file():
        raise EyesOfLunaInferenceError(f"checkpoint missing: {model_path}")
    selected_device = _resolve_device(torch, device)
    try:
        checkpoint = torch.load(model_path, map_location="cpu", weights_only=True)
    except Exception as exc:
        raise EyesOfLunaInferenceError(f"cannot load checkpoint: {exc}") from exc
    if not isinstance(checkpoint, dict) or checkpoint.get("format") != "eyes_of_luna_checkpoint_v1":
        raise EyesOfLunaInferenceError("unsupported checkpoint format")
    if checkpoint.get("config_sha256") != config_sha256(config):
        raise EyesOfLunaInferenceError("checkpoint/config checksum mismatch")
    try:
        model = build_model(config)
        model.load_state_dict(checkpoint["state_dict"], strict=True)
        model.to(selected_device).eval()
    except Exception as exc:
        raise EyesOfLunaInferenceError(f"checkpoint/model mismatch: {exc}") from exc
    return model, checkpoint, selected_device


def load_exported_model(bundle_path: Path, device: str = "cpu"):
    """Load the self-contained torch.export artifact and verify its sidecars."""
    try:
        import torch
    except ImportError as exc:
        raise EyesOfLunaInferenceError("PyTorch is required for inference") from exc
    bundle_path = Path(bundle_path)
    try:
        bundle = json.loads(bundle_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise EyesOfLunaInferenceError(f"invalid deployment bundle: {exc}") from exc
    if not isinstance(bundle, dict) or bundle.get("format") != "eyes_of_luna_export_v1":
        raise EyesOfLunaInferenceError("unsupported deployment bundle format")
    model_path = bundle_path.parent / str(bundle.get("model_file", ""))
    config_path = bundle_path.parent / str(bundle.get("config_file", ""))
    if not model_path.is_file() or not config_path.is_file():
        raise EyesOfLunaInferenceError("deployment model or config sidecar is missing")
    if file_sha256(model_path) != bundle.get("model_sha256"):
        raise EyesOfLunaInferenceError("deployment model checksum mismatch")
    if file_sha256(config_path) != bundle.get("config_file_sha256"):
        raise EyesOfLunaInferenceError("deployment config file checksum mismatch")
    try:
        config = load_model_config(config_path)
        if config_sha256(config) != bundle.get("config_sha256"):
            raise EyesOfLunaInferenceError("deployment config checksum mismatch")
        selected_device = _resolve_device(torch, device)
        program = torch.export.load(model_path)
        # The ExportedProgram is produced from an eval-mode model; its wrapper
        # intentionally does not implement Module.eval().
        model = program.module().to(selected_device)
    except EyesOfLunaInferenceError:
        raise
    except Exception as exc:
        raise EyesOfLunaInferenceError(f"cannot load exported model: {exc}") from exc
    return model, config, bundle, selected_device


def infer_rgb(model, rgb, config: dict, device: str = "cpu") -> dict:
    """Infer both independent masks from an HxWx3 uint8 RGB array."""
    try:
        import numpy as np
        import torch
        import torch.nn.functional as F
        from PIL import Image
    except ImportError as exc:
        raise EyesOfLunaInferenceError(
            "PyTorch, NumPy, and Pillow are required for inference") from exc
    if not isinstance(rgb, np.ndarray) or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise EyesOfLunaInferenceError("expected an HxWx3 RGB NumPy array")
    if rgb.dtype != np.uint8:
        raise EyesOfLunaInferenceError("RGB input must use uint8 values")
    original_height, original_width = rgb.shape[:2]
    if original_height <= 0 or original_width <= 0:
        raise EyesOfLunaInferenceError("RGB image has empty dimensions")

    target_height, target_width = config["training"]["input_size"]
    resized = np.asarray(
        Image.fromarray(rgb).resize((target_width, target_height), Image.Resampling.BILINEAR),
        dtype=np.float32) / 255.0
    mean = np.asarray(config["normalization"]["mean"], dtype=np.float32)
    std = np.asarray(config["normalization"]["std"], dtype=np.float32)
    tensor = ((resized - mean) / std).transpose(2, 0, 1)[None].copy()
    tensor = torch.from_numpy(tensor.astype(np.float32, copy=False)).to(device)
    if device == "cuda":
        start = torch.cuda.Event(enable_timing=True)
        end = torch.cuda.Event(enable_timing=True)
        start.record()
    else:
        import time
        started = time.perf_counter()
    with torch.no_grad():
        terrain_logits, shadow_logits = model(tensor)
        terrain_logits = F.interpolate(terrain_logits, size=(original_height, original_width),
                                       mode="bilinear", align_corners=False)
        shadow_logits = F.interpolate(shadow_logits, size=(original_height, original_width),
                                      mode="bilinear", align_corners=False)
        terrain_probabilities = terrain_logits.softmax(dim=1)[0]
        shadow_probabilities = shadow_logits.sigmoid()[0, 0]
        terrain_confidence, terrain_mask = terrain_probabilities.max(dim=0)
        threshold = float(config["evaluation"]["shadow_threshold"])
        shadow_mask = shadow_probabilities >= threshold
    if device == "cuda":
        end.record()
        torch.cuda.synchronize()
        latency_ms = float(start.elapsed_time(end))
    else:
        latency_ms = (time.perf_counter() - started) * 1000.0
    return {
        "terrain_mask": terrain_mask.cpu().numpy().astype(np.uint8),
        "terrain_confidence": terrain_confidence.cpu().numpy().astype(np.float32),
        "shadow_mask": shadow_mask.cpu().numpy().astype(np.uint8),
        "shadow_probability": shadow_probabilities.cpu().numpy().astype(np.float32),
        "latency_ms": latency_ms,
    }


def save_prediction_masks(output_prefix: Path, terrain_mask, shadow_mask) -> dict:
    """Write class-ID PNGs plus a non-machine-readable shadow preview."""
    try:
        import numpy as np
        from PIL import Image
    except ImportError as exc:
        raise EyesOfLunaInferenceError("NumPy and Pillow are required to save masks") from exc
    terrain = np.asarray(terrain_mask)
    shadow = np.asarray(shadow_mask)
    if terrain.ndim != 2 or shadow.ndim != 2 or terrain.shape != shadow.shape:
        raise EyesOfLunaInferenceError("prediction masks must be same-sized 2D arrays")
    if not np.isin(terrain, (0, 1, 2, 3)).all():
        raise EyesOfLunaInferenceError("terrain output must contain class IDs 0 through 3")
    if not np.isin(shadow, (0, 1)).all():
        raise EyesOfLunaInferenceError("shadow output must contain class IDs 0 or 1")
    prefix = Path(output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    paths = {
        "terrain_mask": prefix.with_name(prefix.name + ".terrain.png"),
        "shadow_mask": prefix.with_name(prefix.name + ".shadow.png"),
        "shadow_visualization": prefix.with_name(prefix.name + ".shadow_visualization.png"),
    }
    Image.fromarray(terrain.astype(np.uint8), mode="L").save(paths["terrain_mask"])
    Image.fromarray(shadow.astype(np.uint8), mode="L").save(paths["shadow_mask"])
    Image.fromarray(shadow.astype(np.uint8) * 255, mode="L").save(
        paths["shadow_visualization"])
    return {name: str(path) for name, path in paths.items()}
