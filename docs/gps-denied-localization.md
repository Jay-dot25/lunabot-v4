# GPS-denied localization

Phase 8 adds `lunabot_localization`, configuring `robot_localization` to fuse
wheel odometry and IMU only. The EKF publishes `odom -> chassis` and
`/odometry/filtered`; slam_toolbox alone publishes `map -> odom`. Ground truth
is never an estimator input.

```bash
ros2 launch lunabot_localization localization.launch.py
ros2 run lunabot_localization localization_evaluator --ros-args \
  -p report_path:=results/localization.json
```

The evaluator synchronizes filtered odometry with the evaluation-only
`/lunabot/ground_truth/pose` and reports ATE RMSE, translational RPE RMSE, final
pose error, final drift per metre, and detected correction jumps. Ground truth
is read only by this evaluator and is not republished into navigation.

The simulator includes Gaussian IMU and lidar noise. Wheel-slip and sensor-bias
magnitudes remain scenario/runtime tuning parameters requiring target Gazebo
verification; the static model does not claim scientifically calibrated lunar
soil slip. Record held-out missions with injected slip/drift and retain exact
noise settings with each report.

Suggested initial evidence thresholds are ATE RMSE <= 0.50 m and final drift <=
2% of travelled distance, to be revised using mission-scale evidence. The exit
gate requires multiple held-out missions; no such physical-performance claim is
made by static configuration tests.
