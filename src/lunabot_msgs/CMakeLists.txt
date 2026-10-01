cmake_minimum_required(VERSION 3.8)
project(lunabot_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TerrainPrediction.msg"
  "msg/PlannerStatus.msg"
  "msg/ReplanEvent.msg"
  "msg/SafetyStatus.msg"
  "msg/MissionMetrics.msg"
  DEPENDENCIES builtin_interfaces sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
