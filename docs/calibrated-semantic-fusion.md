# Calibrated semantic fusion

Phase 9 retains `scripts/semantic_terrain_mapper.py` as the historical baseline
and adds `lunabot_mapping/semantic_fusion_node`. The new mapper synchronizes
class, confidence, depth, and `CameraInfo`, obtains TF at the depth-image stamp,
and uses full pinhole projection into a bounded local map.

```bash
ros2 run lunabot_mapping semantic_fusion_node
```

It publishes aligned local occupancy-grid geometry on:

- `/lunabot/semantic_map/class` (`-1` unknown, otherwise stable class ID)
- `/lunabot/semantic_map/confidence` (posterior confidence, 0–100)

Internally each cell retains semantic evidence, confidence, elevation, and last
observation time. Evidence accumulates rather than using last-write-wins.
Hazard classes 5, 6, and 8 receive conservative evidence weighting and cannot
be erased by subsequent traversable observations. Missing, invalid, out-of-map,
and stale observations remain explicitly unknown.

The map is intentionally a bounded local map in the `map` frame. This limits
loop-closure inconsistency; it does not claim globally persistent semantics
across arbitrary SLAM graph corrections. A future global map must retain source
observations and replay them after pose-graph changes.

Before replacing the baseline, record both outputs on the same representative
bag. Measure class-map IoU against evaluation-only simulator truth and confirm
repeated observations improve posterior confidence. Final numeric IoU remains a
scenario/data-dependent acceptance gate and is not claimed by unit tests.
