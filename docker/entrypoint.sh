#!/bin/bash
set -e

# Source ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Source Gazebo setup if available
if [ -f /usr/share/gazebo/setup.sh ]; then
    source /usr/share/gazebo/setup.sh
fi

# Export TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Export software rendering fallback for Asahi GPU compatibility
export LIBGL_ALWAYS_SOFTWARE=1

# Export Gazebo model path
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH

# Execute the command passed to docker run or drop into bash
exec "$@"
