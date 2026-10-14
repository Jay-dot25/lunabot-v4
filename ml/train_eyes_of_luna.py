#!/usr/bin/env python3
"""Train the Phase 2 dual-head U-Net on reviewed, group-split camera data."""
from __future__ import annotations

import argparse
import json
import random
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lunabot_ml.eyes_of_luna_dataset import (  # noqa: E402
    DatasetError,
    load_capture_group_split,
    validate_dataset,
)
from lunabot_ml.eyes_of_luna_loss import multitask_loss  # noqa: E402
from lunabot_ml.eyes_of_luna_model import build_model, load_model_config  # noqa: E402
from lunabot_ml.eyes_of_luna_runtime import config_sha256, file_sha256  # noqa: E402
from lunabot_ml.eyes_of_luna_torch_dataset import create_dataset  # noqa: E402


def seed_everything(seed: int) -> None:
    random.seed(seed)
    try:
        import numpy as np
        import torch
        np.random.seed(seed)
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(seed)
        torch.use_deterministic_algorithms(True, warn_only=True)
    except ImportError as exc:
        raise RuntimeError("PyTorch and NumPy are required for training") from exc


def _atomic_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent,
                                     prefix=f".{path.name}-", delete=False) as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    temporary.replace(path)


def _resolve_device(torch, requested: str) -> str:
    requested = requested.lower()
    if requested not in ("cpu", "cuda", "auto"):
        raise ValueError("device must be cpu, cuda, or auto")
    if requested == "cuda" and not torch.cuda.is_available():
        raise ValueError("CUDA requested but no CUDA device is available")
    return ("cuda" if requested == "auto" and torch.cuda.is_available()
            else "cpu" if requested == "auto" else requested)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=Path(__file__).resolve().parent / "configs" / "eyes_of_luna.json")
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", choices=("cpu", "cuda", "auto"), default="auto")
    args = parser.parse_args()
    try:
        config = load_model_config(args.config)
        dataset_report = validate_dataset(args.dataset)
        if not dataset_report["valid"]:
            raise DatasetError("dataset validation failed: " + "; ".join(dataset_report["errors"]))
        if not dataset_report["coverage_complete"]:
            raise DatasetError("dataset is missing one or more required terrain/shadow labels")
        if not dataset_report["three_way_split_possible"]:
            raise DatasetError("at least three independent capture groups are required")
        split = load_capture_group_split(args.dataset)
        for name in ("train", "validation", "test"):
            if not split["splits"][name]["samples"]:
                raise DatasetError(f"{name} split is empty")
        import torch
        from torch.utils.data import DataLoader
        device = _resolve_device(torch, args.device)
        seed_everything(config["training"]["seed"])
        train_set = create_dataset(args.dataset, "train", config, training=True)
        validation_set = create_dataset(args.dataset, "validation", config, training=False)
        generator = torch.Generator().manual_seed(config["training"]["seed"])
        common = {
            "batch_size": config["training"]["batch_size"],
            "num_workers": config["training"]["num_workers"],
            "pin_memory": device == "cuda",
        }
        train_loader = DataLoader(train_set, shuffle=True, generator=generator, **common)
        validation_loader = DataLoader(validation_set, shuffle=False, **common)
        model = build_model(config).to(device)
        optimizer = torch.optim.AdamW(
            model.parameters(), lr=config["training"]["learning_rate"],
            weight_decay=config["training"]["weight_decay"])
        output = args.output
        output.mkdir(parents=True, exist_ok=True)
        config_digest = config_sha256(config)
        manifest_digest = file_sha256(args.dataset / "manifest.json")
        split_path = args.dataset / "splits" / "capture_group_split.json"
        split_digest = file_sha256(split_path)
        _atomic_json(output / "config.json", config)
        best_validation = float("inf")
        stale_epochs = 0
        history = []

        for epoch in range(config["training"]["epochs"]):
            model.train()
            train_total = 0.0
            train_batches = 0
            for images, terrain, shadow, _sample_ids in train_loader:
                images = images.to(device)
                terrain = terrain.to(device)
                shadow = shadow.to(device)
                optimizer.zero_grad(set_to_none=True)
                terrain_logits, shadow_logits = model(images)
                loss, _components = multitask_loss(
                    terrain_logits, shadow_logits, terrain, shadow,
                    terrain_dice_weight=config["training"]["terrain_dice_weight"],
                    shadow_loss_weight=config["training"]["shadow_loss_weight"],
                    shadow_dice_weight=config["training"]["shadow_dice_weight"],
                )
                if not torch.isfinite(loss):
                    raise RuntimeError(f"non-finite training loss in epoch {epoch + 1}")
                loss.backward()
                optimizer.step()
                train_total += float(loss.detach().cpu())
                train_batches += 1

            model.eval()
            validation_total = 0.0
            validation_batches = 0
            with torch.no_grad():
                for images, terrain, shadow, _sample_ids in validation_loader:
                    terrain_logits, shadow_logits = model(images.to(device))
                    loss, _components = multitask_loss(
                        terrain_logits, shadow_logits, terrain.to(device), shadow.to(device),
                        terrain_dice_weight=config["training"]["terrain_dice_weight"],
                        shadow_loss_weight=config["training"]["shadow_loss_weight"],
                        shadow_dice_weight=config["training"]["shadow_dice_weight"],
                    )
                    validation_total += float(loss.detach().cpu())
                    validation_batches += 1
            if train_batches == 0 or validation_batches == 0:
                raise DatasetError("train/validation loader produced no batches")
            if not torch.isfinite(torch.tensor(validation_total)):
                raise RuntimeError(f"non-finite validation loss in epoch {epoch + 1}")
            row = {
                "epoch": epoch + 1,
                "train_loss": train_total / train_batches,
                "validation_loss": validation_total / validation_batches,
            }
            history.append(row)
            print(json.dumps(row, sort_keys=True), flush=True)
            if row["validation_loss"] < best_validation:
                best_validation = row["validation_loss"]
                stale_epochs = 0
                checkpoint = {
                    "format": "eyes_of_luna_checkpoint_v1",
                    "state_dict": {key: value.detach().cpu()
                                   for key, value in model.state_dict().items()},
                    "config_sha256": config_digest,
                    "manifest_sha256": manifest_digest,
                    "split_sha256": split_digest,
                    "epoch": epoch + 1,
                    "validation_loss": best_validation,
                }
                temporary = output / ".best.pt.tmp"
                torch.save(checkpoint, temporary)
                temporary.replace(output / "best.pt")
            else:
                stale_epochs += 1
                if stale_epochs >= config["training"]["early_stopping_patience"]:
                    break

        _atomic_json(output / "history.json", {"epochs": history})
        best = torch.load(output / "best.pt", map_location="cpu", weights_only=True)
        model = build_model(config)
        model.load_state_dict(best["state_dict"], strict=True)
        model.eval()
        height, width = config["training"]["input_size"]
        example = torch.zeros(1, 3, height, width, dtype=torch.float32)
        exported = torch.export.export(model, (example,))
        exported_path = output / "best.pt2"
        torch.export.save(exported, exported_path)
        (output / "best.pt.sha256").write_text(
            f"{file_sha256(output / 'best.pt')}  best.pt\n", encoding="utf-8")
        exported_digest = file_sha256(exported_path)
        (output / "best.pt2.sha256").write_text(
            f"{exported_digest}  best.pt2\n", encoding="utf-8")
        _atomic_json(output / "deployment.json", {
            "format": "eyes_of_luna_export_v1",
            "model_file": exported_path.name,
            "model_sha256": exported_digest,
            "config_file": "config.json",
            "config_file_sha256": file_sha256(output / "config.json"),
            "config_sha256": config_digest,
            "checkpoint_sha256": file_sha256(output / "best.pt"),
            "manifest_sha256": manifest_digest,
            "split_sha256": split_digest,
            "selected_epoch": int(best["epoch"]),
            "validation_loss": float(best["validation_loss"]),
        })
        print(f"EYES_OF_LUNA_TRAIN_COMPLETE epoch={best['epoch']} "
              f"validation_loss={best['validation_loss']:.6f}")
        return 0
    except (DatasetError, OSError, ValueError, RuntimeError, KeyError) as exc:
        print(f"EYES_OF_LUNA_TRAIN_FAIL: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
