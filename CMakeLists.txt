cmake_minimum_required(VERSION 3.8)
project(lunabot_v4)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/wasd_teleop.py
  scripts/control_odometry.py
  scripts/odometry_monitor.py
  scripts/astar_navigation.py
  scripts/terrain_segmentation.py
  scripts/semantic_terrain_mapper.py
  scripts/terrain_cost_mapper.py
  scripts/terrain_aware_planner.py
  scripts/terrain_path_follower.py
  scripts/dynamic_replan_monitor.py
  scripts/phase_k_evaluator.py
  scripts/obstacle_detector.py
  scripts/phase_l_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  docs
  launch
  rviz
  src/lunabot_gazebo
  tools
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
