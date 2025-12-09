#!/bin/bash
set -e

# ROS
source /opt/ros/noetic/setup.bash

# 若 rosdep 缓存不存在，则更新（避免每次都更新）
if [ ! -d "/root/.ros/rosdep" ]; then
    rosdep update || echo "rosdep update failed, continue..."
fi

exec "$@"

