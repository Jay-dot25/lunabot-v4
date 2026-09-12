# Phase A evidence

## Already generated (in this directory)

| File | What it is |
|---|---|
| `terrain_preview_topdown.png` | top-down render of the generated lunar terrain mesh (400 m × 400 m), large craters outlined, spawn pad marked |
| `terrain_preview_perspective.png` | perspective render from ~ the Gazebo GUI camera position |
| `terrain_stats.txt` | deterministic terrain stats: extent, grid, relief, crater counts, **spawn height** |
| `static_validation.txt` | full static cross-validation (SDF semantics, mesh integrity, topic contract, script syntax, docs) |
| `verification_checklist.md` | visual acceptance checklist (user gatekeeper) |
| `collect_evidence.sh` | one command that regenerates all **runtime** evidence |

## Runtime evidence (needs a machine with ROS 2 Humble + Gazebo Sim)

Run:

```bash
EVIDENCE=1 DEMO=1 ~/launch-a
```

or `bash evidence/phase-a-launch-a/collect_evidence.sh` from the repo root.
This produces, in this directory:

| File | What it is |
|---|---|
| `last_run.log` | full terminal transcript of the launch (all 10 stages + validation) |
| `gazebo.log` | Gazebo Sim log (world load, sensors, spawn) |
| `bridge.log` | ros_ign bridge log (15 topic mappings) |
| `topics.txt` | `ros2 topic list` |
| `topic_info.txt` | `ros2 topic info --verbose` for every topic |
| `tf_odom_chassis.txt` | first `odom → chassis` transforms (tf2_echo) |
| `odom_sample.txt` | one odometry message (YAML) |
| `lidar_scan_sample.yaml` | one LiDAR scan (YAML) |
| `lidar_scan_preview.png` | top-down plot of the scan (crater field around rover) — needs `sudo apt install python3-matplotlib python3-yaml` |
| `demo_drive_result.txt` | automated drive test (sim-time based): distance driven + yaw change + efficiency vs ideal (PASS/FAIL) |
| `diag_drive.csv` | per-sample drive diagnostics: sim time, commanded vs measured velocity, rover pose, actual wheel joint velocities (left/right averages) — the key data for separating wheel slip/traction from sim-time lag |

## Why there are no Gazebo/RViz screenshots in this directory yet

The development sandbox where Phase A was implemented has **no ROS 2 /
Gazebo** (Debian 12 with a restricted network: no package mirrors
reachable, no container runtime) — the runtime steps in
`docs/phase-a-launch-a.md` §15/§16 must be executed on the workstation.
The static evidence above is real and complete; the runtime files appear
after the first `EVIDENCE=1` run on a machine with the stack installed.
No placeholder/fake evidence has been created.
