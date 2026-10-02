# ROS ML terrain inference

`terrain_inference_node` synchronizes RGB images and camera calibration, applies
the exact normalization and size from the exported model config, executes ONNX
Runtime on CPU or CUDA, thresholds low-confidence pixels to class 0 (unknown),
and publishes aligned class, confidence, overlay, and typed status outputs.

```bash
ros2 launch lunabot_bringup perception.launch.py perception_mode:=ml \
  model_path:=/models/terrain_segmentation.onnx \
  model_config:=/models/terrain_segmentation.config.json \
  model_checksum:=/models/terrain_segmentation.sha256 \
  inference_device:=auto
```

Modes are `heuristic`, `ml`, and `ground_truth`. Ground truth is strictly for
debugging/evaluation and is forbidden in final navigation results. An absent,
tampered, incompatible, or unloadable model leaves the node alive, emits an
ERROR `TerrainPrediction`, and publishes no fabricated class map.

Inputs are `/lunabot/camera/image_raw` and
`/lunabot/camera/camera_info`. The selected Phase 6 model is RGB-only, so depth
is not synchronized. A future RGB-D model must add depth to the synchronizer
and declare that input in its model contract.

Outputs:

- `/lunabot/terrain/class_image` (`mono8`)
- `/lunabot/terrain/confidence` (`32FC1`)
- `/lunabot/terrain/overlay` (`rgb8`)
- `/lunabot/terrain/inference_status` (`lunabot_msgs/TerrainPrediction`)

The ML environment pins NumPy below 2 because ROS 2 Humble's packaged
`cv_bridge` extension uses the NumPy 1.x ABI. After changing requirements, run
`.venv-ml/bin/pip install -r ml/requirements.txt` before starting this node.

The software gate checks contracts and deterministic offline execution. The
final live FPS and rosbag agreement gate requires a trained production model,
target machine, and representative bag; no such performance claim is made by
the random-weight smoke artifact.
