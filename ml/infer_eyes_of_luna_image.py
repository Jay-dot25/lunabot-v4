#!/usr/bin/env python3
"""Run standalone dual-head inference on one RGB image."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lunabot_ml.eyes_of_luna_model import load_model_config  # noqa: E402
from lunabot_ml.eyes_of_luna_runtime import (  # noqa: E402
    EyesOfLunaInferenceError,
    config_sha256,
    file_sha256,
    infer_rgb,
    load_checkpoint,
    save_prediction_masks,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=Path(__file__).resolve().parent / "configs" / "eyes_of_luna.json")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--image-id", help="Preserved source ID; defaults to the image filename stem.")
    parser.add_argument("--output-prefix", type=Path, required=True)
    parser.add_argument("--device", choices=("cpu", "cuda", "auto"), default="auto")
    args = parser.parse_args()
    try:
        import numpy as np
        from PIL import Image
        config = load_model_config(args.config)
        model, checkpoint, device = load_checkpoint(args.checkpoint, config, args.device)
        with Image.open(args.image) as source:
            rgb = np.asarray(source.convert("RGB"), dtype=np.uint8)
        result = infer_rgb(model, rgb, config, device)
        prefix = args.output_prefix
        prefix.parent.mkdir(parents=True, exist_ok=True)
        mask_paths = save_prediction_masks(
            prefix, result["terrain_mask"], result["shadow_mask"])
        confidence_path = prefix.with_name(prefix.name + ".terrain_confidence.png")
        shadow_probability_path = prefix.with_name(prefix.name + ".shadow_probability.png")
        report_path = prefix.with_name(prefix.name + ".json")
        terrain_confidence = np.clip(result["terrain_confidence"] * 255.0, 0, 255).astype(np.uint8)
        shadow_probability = np.clip(result["shadow_probability"] * 255.0, 0, 255).astype(np.uint8)
        Image.fromarray(terrain_confidence).save(confidence_path)
        Image.fromarray(shadow_probability).save(shadow_probability_path)
        height, width = result["terrain_mask"].shape
        counts = np.bincount(result["terrain_mask"].reshape(-1), minlength=4)
        report = {
            "schema_version": 1,
            "task": "eyes_of_luna",
            "input": str(args.image),
            "image_id": args.image_id or args.image.stem,
            "width": int(width),
            "height": int(height),
            "checkpoint": str(args.checkpoint),
            "checkpoint_sha256": file_sha256(args.checkpoint),
            "config_sha256": config_sha256(config),
            "device": device,
            "terrain_pixel_counts": {str(index): int(value)
                                     for index, value in enumerate(counts[:4])},
            "shadow_pixel_counts": {
                "0": int((result["shadow_mask"] == 0).sum()),
                "1": int((result["shadow_mask"] == 1).sum()),
            },
            "mean_terrain_confidence": float(result["terrain_confidence"].mean()),
            "mean_shadow_probability": float(result["shadow_probability"].mean()),
            "outputs": {
                **mask_paths,
                "terrain_confidence": str(confidence_path),
                "shadow_probability": str(shadow_probability_path),
            },
            "inference_time_ms": float(result["latency_ms"]),
            "frames_per_second": 1000.0 / result["latency_ms"]
            if result["latency_ms"] > 0 else 0.0,
            "warning": "Predictions are not quality evidence without held-out evaluation.",
        }
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n",
                               encoding="utf-8")
        print(json.dumps(report, indent=2, sort_keys=True))
        print("EYES_OF_LUNA_INFERENCE_PASS")
        return 0
    except (EyesOfLunaInferenceError, ImportError, OSError, ValueError, RuntimeError, KeyError) as exc:
        print(f"EYES_OF_LUNA_INFERENCE_FAIL: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
