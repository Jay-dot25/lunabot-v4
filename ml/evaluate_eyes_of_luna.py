#!/usr/bin/env python3
"""Evaluate a reloadable checkpoint only on the held-out capture-group test split."""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lunabot_ml.eyes_of_luna_dataset import (  # noqa: E402
    DatasetError,
    capture_group_split_coverage,
    load_capture_group_split,
    validate_dataset,
)
from lunabot_ml.eyes_of_luna_metrics import (  # noqa: E402
    confusion_matrix,
    merge_confusion,
    shadow_report,
    terrain_report,
)
from lunabot_ml.eyes_of_luna_model import load_model_config  # noqa: E402
from lunabot_ml.eyes_of_luna_runtime import (  # noqa: E402
    EyesOfLunaInferenceError,
    file_sha256,
    load_checkpoint,
    save_prediction_masks,
)
from lunabot_ml.eyes_of_luna_torch_dataset import create_dataset  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=Path(__file__).resolve().parent / "configs" / "eyes_of_luna.json")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--predictions-dir", type=Path)
    parser.add_argument("--device", choices=("cpu", "cuda", "auto"), default="auto")
    args = parser.parse_args()
    try:
        config = load_model_config(args.config)
        dataset_report = validate_dataset(args.dataset)
        if not dataset_report["valid"]:
            raise DatasetError("dataset validation failed: " + "; ".join(dataset_report["errors"]))
        split = load_capture_group_split(args.dataset)
        if not split["splits"]["test"]["samples"]:
            raise DatasetError("held-out test split is empty")
        test_coverage = capture_group_split_coverage(args.dataset, split)["test"]
        if not test_coverage["complete"]:
            raise DatasetError(
                "held-out test split lacks class coverage: "
                + json.dumps({
                    "missing_terrain_classes": test_coverage["missing_terrain_classes"],
                    "missing_shadow_classes": test_coverage["missing_shadow_classes"],
                }, sort_keys=True))
        try:
            import numpy as np
            import torch
        except ImportError as exc:
            raise RuntimeError("NumPy and PyTorch are required for evaluation") from exc
        model, checkpoint, device = load_checkpoint(args.checkpoint, config, args.device)
        manifest_hash = file_sha256(args.dataset / "manifest.json")
        split_hash = file_sha256(args.dataset / "splits" / "capture_group_split.json")
        if checkpoint.get("manifest_sha256") != manifest_hash:
            raise EyesOfLunaInferenceError("checkpoint was trained against a different dataset manifest")
        if checkpoint.get("split_sha256") != split_hash:
            raise EyesOfLunaInferenceError("checkpoint was trained against a different split file")

        dataset = create_dataset(args.dataset, "test", config, training=False)
        loader = torch.utils.data.DataLoader(dataset, batch_size=1, shuffle=False, num_workers=0)
        terrain_matrix = [[0] * 4 for _ in range(4)]
        shadow_matrix = [[0] * 2 for _ in range(2)]
        predictions_dir = args.predictions_dir or args.output.parent / "eyes_of_luna_test_predictions"
        predictions_dir.mkdir(parents=True, exist_ok=True)
        threshold = float(config["evaluation"]["shadow_threshold"])
        total_seconds = 0.0
        tested = 0
        prediction_records = []
        model.eval()
        with torch.no_grad():
            for images, terrain, shadow, sample_ids in loader:
                images = images.to(device)
                if device == "cuda":
                    torch.cuda.synchronize()
                started = time.perf_counter()
                terrain_logits, shadow_logits = model(images)
                if device == "cuda":
                    torch.cuda.synchronize()
                total_seconds += time.perf_counter() - started
                terrain_prediction = terrain_logits.argmax(dim=1).cpu().numpy().astype(np.uint8)
                shadow_prediction = (shadow_logits[:, 0].sigmoid() >= threshold).cpu().numpy().astype(np.uint8)
                terrain_target = terrain.numpy().astype(np.int64)
                shadow_target = shadow.numpy().astype(np.int64)
                terrain_matrix = merge_confusion(
                    terrain_matrix,
                    confusion_matrix(terrain_prediction[0].reshape(-1).tolist(),
                                     terrain_target[0].reshape(-1).tolist(), 4))
                shadow_matrix = merge_confusion(
                    shadow_matrix,
                    confusion_matrix(shadow_prediction[0].reshape(-1).tolist(),
                                     shadow_target[0].reshape(-1).tolist(), 2))
                sample_id = str(sample_ids[0])
                saved_paths = save_prediction_masks(
                    predictions_dir / sample_id,
                    terrain_prediction[0],
                    shadow_prediction[0],
                )
                prediction_records.append({"sample_id": sample_id, **saved_paths})
                tested += 1
        result = {
            "schema_version": 1,
            "task": "eyes_of_luna",
            "split": "test",
            "test_samples": tested,
            "test_capture_groups": split["splits"]["test"]["groups"],
            "terrain": terrain_report(terrain_matrix),
            "shadow": shadow_report(shadow_matrix),
            "inference_time_ms_per_image": 1000.0 * total_seconds / max(1, tested),
            "inference_fps": tested / total_seconds if total_seconds else 0.0,
            "device": device,
            "checkpoint_sha256": file_sha256(args.checkpoint),
            "manifest_sha256": manifest_hash,
            "split_sha256": split_hash,
            "predictions_dir": str(predictions_dir),
            "predictions": prediction_records,
            "note": "Metrics apply only to these held-out capture groups and annotations.",
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                               encoding="utf-8")
        print(json.dumps(result, indent=2, sort_keys=True))
        print("EYES_OF_LUNA_EVALUATION_COMPLETE")
        return 0
    except (DatasetError, EyesOfLunaInferenceError, OSError, ValueError,
            RuntimeError, KeyError) as exc:
        print(f"EYES_OF_LUNA_EVALUATION_FAIL: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
