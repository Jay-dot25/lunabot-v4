# Phase I — Full Autonomous Integration (`launch-i`) — Terrain-aware autonomy

## 1. Scope and gate

Phase I integrates terrain-aware plan into active autonomy via terrain_path_follower -> /cmd_vel_in -> B controller -> /cmd_vel.

Single velocity boundary: /cmd_vel_in -> controller -> /cmd_vel, independent launchers, no competing publishers.

Phase I preserves diagnostic A* on /lunabot/navigation/diagnostic_cmd_vel for isolation.

Publishes:
 /lunabot/autonomy/status (INTEGRATION_GOAL_REACHED)

## 2. Data flow

```
/lunabot/terrain/plan (graded cost-aware) -> terrain_path_follower -> /cmd_vel_in -> control_odometry.py -> /cmd_vel
TF map->chassis, /lunabot/odom, /goal_pose
```

## 3. Interface

| Interface | Type |
|---|---|
| `/lunabot/terrain/plan` | Path |
| `/cmd_vel_in` | Twist (only motion source) |
| `/cmd_vel` | Twist (controller output) |
| `/lunabot/autonomy/status` | String INTEGRATION_GOAL_REACHED |

cost_weight 2.5 preserved.

## 4. Installation

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-i" ~/launch-i
chmod +x launch-i scripts/launch-i.sh scripts/terrain_path_follower.py
```

## 5. Static validation

```bash
python3 tools/validate_phase_i.py
```

Checks follower consumes terrain path, publishes into controller boundary /cmd_vel_in, conservative limits, no final /cmd_vel, cost_weight 2.5.

## 6. Automated headless

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-i
```

Validates integration status INTEGRATION_GOAL_REACHED, controller boundary motion.

## 7. GUI

EVIDENCE=1 ~/launch-i shows Active Terrain Path.

## 8. Evidence

evidence/phase-i-launch-i/: integration.log, integration_status.txt, cmd_vel_in_motion.txt, cmd_vel_motion.txt, etc.

## 9. Clean relaunch

Launch I environment cleanly closed. Second run.

## 10. Acceptance

- [ ] Active autonomy via terrain_path_follower -> /cmd_vel_in -> B controller
- [ ] Single velocity boundary
- [ ] cost_weight 2.5 graded
