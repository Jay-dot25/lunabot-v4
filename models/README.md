# Model artifacts

Large trained weights are intentionally not committed. Phase 6 commands write
`terrain_segmentation.onnx`, `terrain_segmentation.config.json`, and
`terrain_segmentation.sha256` here. Publish production artifacts through the
project's release/object-storage process and verify the checksum before use.
No accuracy or deployment-performance claim is made until evaluation on the
held-out world split and target hardware passes the documented thresholds.
