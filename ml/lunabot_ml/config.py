"""Strict, reproducible configuration handling for terrain ML runs."""
from __future__ import annotations
import hashlib, json, random
from pathlib import Path

REQUIRED = {"schema_version", "model", "data", "training", "normalization", "classes"}


def load_config(path: Path) -> dict:
    """Load JSON-compatible YAML without requiring PyYAML."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    missing = REQUIRED - data.keys()
    if missing:
        raise ValueError("config missing: " + ", ".join(sorted(missing)))
    if data["schema_version"] != 1 or data["classes"] != list(range(9)):
        raise ValueError("unsupported schema or class IDs")
    if data["model"].get("architecture") not in {"unet", "deeplabv3plus", "segformer_b0"}:
        raise ValueError("unsupported architecture")
    return data


def canonical_json(config: dict) -> str:
    return json.dumps(config, sort_keys=True, separators=(",", ":"))


def config_sha256(config: dict) -> str:
    return hashlib.sha256(canonical_json(config).encode()).hexdigest()


def seed_everything(seed: int) -> None:
    random.seed(seed)
    try:
        import numpy as np
        np.random.seed(seed)
        import torch
        torch.manual_seed(seed)
        torch.cuda.manual_seed_all(seed)
        torch.use_deterministic_algorithms(True, warn_only=True)
    except ImportError:
        pass
