# Terrain model training, evaluation, and export

Phase 6 provides a reproducible PyTorch framework for U-Net, a compact
DeepLabV3+-style model, and SegFormer-B0-style transformer baseline. It does not
claim trained-model quality: the repository has no qualifying 5,000-sample,
30-world dataset or deployment hardware benchmark.

## Environment

```bash
python3 -m venv .venv-ml
.venv-ml/bin/pip install -r ml/requirements.txt
```

Create the leakage-free split first with `ml/split_dataset.py`. Configuration
files are JSON-compatible YAML so they remain readable without PyYAML. Each run
uses deterministic seeds and saves its exact config and config hash.

## Train and evaluate

```bash
.venv-ml/bin/python ml/train.py --config ml/configs/unet.yaml \
  --dataset /data/lunabot --output results/unet --device cuda
.venv-ml/bin/python ml/evaluate.py --config results/unet/config.json \
  --checkpoint results/unet/best.pt --dataset /data/lunabot \
  --output results/unet/test-report.json --device cuda
```

Evaluation is hard-coded to the held-out `test` world split and reports the
confusion matrix, per-class IoU/precision/recall/F1, mIoU, combined large-rock
and crater false-negative rate, latency, and FPS. Preserve representative
success/failure images alongside the report during the final experiment.

## Export and smoke inference

```bash
.venv-ml/bin/python ml/export_onnx.py --config results/unet/config.json \
  --checkpoint results/unet/best.pt --output models/terrain_segmentation.onnx
.venv-ml/bin/python ml/infer_image.py \
  --model models/terrain_segmentation.onnx \
  --config models/terrain_segmentation.config.json \
  --image sample.png --output prediction.png
sha256sum -c models/terrain_segmentation.sha256
```

The exporter embeds weights in one self-contained ONNX file rather than an
unchecked external-data sidecar. Do not commit large weights. Release them
through object storage with the ONNX file, exact config, checksum, evaluation
report, dependency lock/environment, and dataset-manifest checksum.

## Acceptance boundary

A production artifact is accepted only when the held-out-world report shows
mIoU >= 0.65, recall >= 0.90 separately for classes 5 (large rock) and 6
(crater), and >= 10 FPS on deployment hardware, with no world leakage. Those
compute/data-dependent gates have not been claimed by this implementation.
