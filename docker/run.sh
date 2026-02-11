#!/bin/bash

# Convenience script to run the ROS 2 Humble + TurtleBot3 Docker environment
# with proper X11 GUI support

set -e

# Change to the docker directory
cd "$(dirname "$0")"

echo "Setting up X11 access for Docker containers..."
xhost +local:docker

# Function to cleanup on exit
cleanup() {
    echo "Cleaning up X11 access..."
    xhost -local:docker
}

# Register cleanup function
trap cleanup EXIT

echo "Starting ROS 2 Humble + TurtleBot3 Docker environment..."
echo "Use 'docker exec -it ros2_humble_turtlebot3 bash' to open additional terminals."
echo ""

# Try docker compose (new) first, fall back to docker-compose (old)
if command -v docker &> /dev/null && docker compose version &> /dev/null; then
    docker compose up
elif command -v docker-compose &> /dev/null; then
    docker-compose up
else
    echo "Error: Neither 'docker compose' nor 'docker-compose' found."
    echo "Please install Docker Compose."
    exit 1
fi
