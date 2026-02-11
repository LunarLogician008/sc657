#!/bin/bash
set -e

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source Gazebo ROS packages (built from source)
if [ -f /opt/gazebo_ros_ws/install/setup.bash ]; then
    source /opt/gazebo_ros_ws/install/setup.bash
fi

# Source TurtleBot3 simulation packages (built from source)
if [ -f /opt/tb3_sim_ws/install/setup.bash ]; then
    source /opt/tb3_sim_ws/install/setup.bash
fi

# Source user workspace if built
if [ -f /home/rosuser/ros2_ws/install/setup.bash ]; then
    source /home/rosuser/ros2_ws/install/setup.bash
fi

# Source Gazebo setup if available
if [ -f /usr/share/gazebo/setup.sh ]; then
    source /usr/share/gazebo/setup.sh
fi

export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:/opt/tb3_sim_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH:-}

exec "$@"
