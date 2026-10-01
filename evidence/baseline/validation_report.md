# LunaBot Baseline Validation Report

- Generated: `2026-10-01T13:21:24.392127+00:00`
- Commit: `6fe435a8a5fa5af7f209e34c7c5c8caa166d4088`
- Branch: `arena/01a0f77b-lunabot-v4`
- Result: **PASS**
- Checks: 68 passed, 0 failed, 0 skipped

## Checks

| Status | Category | Check | Detail |
|---|---|---|---|
| PASS | python | `scripts/astar_navigation.py` | syntax and bytecode compilation succeeded |
| PASS | python | `scripts/control_odometry.py` | syntax and bytecode compilation succeeded |
| PASS | python | `scripts/dynamic_replan_monitor.py` | syntax and bytecode compilation succeeded |
| PASS | python | `scripts/obstacle_detector.py` | syntax and bytecode compilation succeeded |
| PASS | python | `scripts/odometry_monitor.py` | syntax and bytecode compilation succeeded |
| PASS | python | `scripts/phase_k_evaluator.py` | syntax and bytecode compilation succeeded |
| PASS | python | `scripts/phase_l_mission.py` | syntax and bytecode compilation succeeded |
| PASS | python | `scripts/semantic_terrain_mapper.py` | syntax and bytecode compilation succeeded |
| PASS | python | `scripts/terrain_aware_planner.py` | syntax and bytecode compilation succeeded |
| PASS | python | `scripts/terrain_cost_mapper.py` | syntax and bytecode compilation succeeded |
| PASS | python | `scripts/terrain_path_follower.py` | syntax and bytecode compilation succeeded |
| PASS | python | `scripts/terrain_segmentation.py` | syntax and bytecode compilation succeeded |
| PASS | python | `scripts/wasd_teleop.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tests/test_phase0_validation.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/check_environment.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/generate_lunar_terrain.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/plot_lidar_scan.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/validate_all.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/validate_phase_a.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/validate_phase_b.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/validate_phase_c.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/validate_phase_d.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/validate_phase_e.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/validate_phase_f.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/validate_phase_g.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/validate_phase_h.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/validate_phase_i.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/validate_phase_j.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/validate_phase_k.py` | syntax and bytecode compilation succeeded |
| PASS | python | `tools/validate_phase_l.py` | syntax and bytecode compilation succeeded |
| PASS | shell | `launch-a` | bash -n succeeded |
| PASS | shell | `launch-b` | bash -n succeeded |
| PASS | shell | `launch-c` | bash -n succeeded |
| PASS | shell | `launch-d` | bash -n succeeded |
| PASS | shell | `launch-e` | bash -n succeeded |
| PASS | shell | `launch-f` | bash -n succeeded |
| PASS | shell | `launch-g` | bash -n succeeded |
| PASS | shell | `launch-h` | bash -n succeeded |
| PASS | shell | `launch-i` | bash -n succeeded |
| PASS | shell | `launch-j` | bash -n succeeded |
| PASS | shell | `launch-k` | bash -n succeeded |
| PASS | shell | `launch-l` | bash -n succeeded |
| PASS | shell | `scripts/launch-a.sh` | bash -n succeeded |
| PASS | shell | `scripts/launch-b.sh` | bash -n succeeded |
| PASS | shell | `scripts/launch-c.sh` | bash -n succeeded |
| PASS | shell | `scripts/launch-d.sh` | bash -n succeeded |
| PASS | shell | `scripts/launch-e.sh` | bash -n succeeded |
| PASS | shell | `scripts/launch-f.sh` | bash -n succeeded |
| PASS | shell | `scripts/launch-g.sh` | bash -n succeeded |
| PASS | shell | `scripts/launch-h.sh` | bash -n succeeded |
| PASS | shell | `scripts/launch-i.sh` | bash -n succeeded |
| PASS | shell | `scripts/launch-j.sh` | bash -n succeeded |
| PASS | shell | `scripts/launch-k.sh` | bash -n succeeded |
| PASS | shell | `scripts/launch-l.sh` | bash -n succeeded |
| PASS | xml | `src/lunabot_gazebo/models/lunabot_v4/model.sdf` | XML is well formed |
| PASS | xml | `src/lunabot_gazebo/worlds/lunar_world.sdf` | XML is well formed |
| PASS | legacy-validator | `tools/validate_phase_a.py` | exit=0; tail: [PASS] docs section ## 23. Known Limitations \| [PASS] terrain stats file \| [PASS] evidence README \| [PASS] verification checklist \| ============================================================ \| RESULT: 80/80 checks passed - ALL PASS \| ============================================================ \| report: /home/user/lunabot-v4/evidence/phase-a-launch-a/static_validation.txt |
| PASS | legacy-validator | `tools/validate_phase_b.py` | exit=0; tail: [PASS] Phase A static baseline remains 80/80 - ===================== \| RESULT: 80/80 checks passed - ALL PASS \| ============================================================ \| report: /home/user/lunabot-v4/evidence/phase-a-launch-a/static_validation.txt \| ============================================================ \| RESULT: 103/103 checks passed - ALL PASS \| ============================================================ \| report: /home/user/lunabot-v4/evidence/phase-b-launch-b/static_validation.txt |
| PASS | legacy-validator | `tools/validate_phase_c.py` | exit=0; tail: [PASS] docs contain IMU acceptance \| [PASS] docs contain relaunch instruction \| [PASS] docs contain Phase D gate \| [PASS] validator states that static checks are not runtime acceptance \| ================================================================ \| RESULT: 133/133 checks passed - ALL PASS \| ================================================================ \| report: /home/user/lunabot-v4/evidence/phase-c-launch-c/static_validation.txt |
| PASS | legacy-validator | `tools/validate_phase_d.py` | exit=0; tail: [PASS] docs contain map evidence \| [PASS] docs contain relaunch \| [PASS] docs contain Phase E gate \| [PASS] validator is honest about runtime \| ================================================================ \| RESULT: 152/152 checks passed - ALL PASS \| ================================================================ \| report: /home/user/lunabot-v4/evidence/phase-d-launch-d/static_validation.txt |
| PASS | legacy-validator | `tools/validate_phase_e.py` | exit=0; tail: [PASS] docs contain RGB-D inputs \| [PASS] docs contain relaunch \| [PASS] docs contain Phase F gate \| [PASS] validator is honest about runtime \| ================================================================ \| RESULT: 112/112 checks passed - ALL PASS \| ================================================================ \| report: /home/user/lunabot-v4/evidence/phase-e-launch-e/static_validation.txt |
| PASS | legacy-validator | `tools/validate_phase_f.py` | exit=0; tail: [PASS] docs contain legend \| [PASS] docs contain relaunch \| [PASS] docs contain Phase G gate \| [PASS] validator is honest about runtime \| ================================================================ \| RESULT: 122/122 checks passed - ALL PASS \| ================================================================ \| report: /home/user/lunabot-v4/evidence/phase-f-launch-f/static_validation.txt |
| PASS | legacy-validator | `tools/validate_phase_g.py` | exit=0; tail: [PASS] docs contain obstacle inflation \| [PASS] docs contain relaunch \| [PASS] docs contain Phase H gate \| [PASS] validator is honest about runtime \| ================================================================ \| RESULT: 131/131 checks passed - ALL PASS \| ================================================================ \| report: /home/user/lunabot-v4/evidence/phase-g-launch-g/static_validation.txt |
| PASS | legacy-validator | `tools/validate_phase_h.py` | exit=0; tail: [PASS] docs contain motion isolation \| [PASS] docs contain relaunch \| [PASS] docs contain Phase I gate \| [PASS] validator is honest about runtime \| ================================================================ \| RESULT: 144/144 checks passed - ALL PASS \| ================================================================ \| report: /home/user/lunabot-v4/evidence/phase-h-launch-h/static_validation.txt |
| PASS | legacy-validator | `tools/validate_phase_i.py` | exit=0; tail: [PASS] docs contain phase H distinction \| [PASS] docs contain relaunch \| [PASS] docs contain Phase J gate \| [PASS] validator is honest about runtime \| ================================================================ \| RESULT: 160/160 checks passed - ALL PASS \| ================================================================ \| report: /home/user/lunabot-v4/evidence/phase-i-launch-i/static_validation.txt |
| PASS | legacy-validator | `tools/validate_phase_j.py` | exit=0; tail: [PASS] docs contain status output \| [PASS] docs contain revision requirement \| [PASS] docs contain relaunch \| [PASS] docs contain Phase K gate \| [PASS] validator is honest about runtime \| ================================================================ \| RESULT: 99/99 checks passed - ALL PASS \| ================================================================ |
| PASS | legacy-validator | `tools/validate_phase_k.py` | exit=0; tail: [PASS] docs contain status output \| [PASS] docs contain boundary metrics \| [PASS] docs contain relaunch \| [PASS] docs contain Phase L gate \| [PASS] validator is honest about runtime \| ================================================================ \| RESULT: 105/105 checks passed - ALL PASS \| ================================================================ |
| PASS | legacy-validator | `tools/validate_phase_l.py` | exit=0; tail: [PASS] docs contain status output \| [PASS] docs contain safety boundary \| [PASS] docs contain relaunch \| [PASS] docs contain final-phase gate \| [PASS] validator is honest about runtime \| ================================================================ \| RESULT: 105/105 checks passed - ALL PASS \| ================================================================ |

## Meaning

This report proves repository-level static integrity only. It does not claim that ROS 2, Gazebo, SLAM, navigation, or the final mission ran on this machine. Runtime acceptance must be performed on a workstation that passes `python3 tools/check_environment.py`.
