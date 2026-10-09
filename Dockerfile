# syntax=docker/dockerfile:1
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-slam-toolbox \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-tf2-ros \
    ros-humble-rviz2 \
    python3-numpy \
    python3-matplotlib \
    python3-pil \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/lunabot-v4
COPY . /workspace/lunabot-v4

RUN python3 -m unittest discover -s tests -v

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 tools/validate_phase_l.py"]
