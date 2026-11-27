#!/usr/bin/env bash
set -eo pipefail
# ROS setup scripts may read unset variables.
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/robot_avoidance_ws/install/setup.bash"
export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1
export PYTHONUNBUFFERED=1
unset WAYLAND_DISPLAY LIBGL_DRI3_DISABLE
exec python3 "$(dirname "$(realpath "$0")")/run_exploration.py" "$@"
