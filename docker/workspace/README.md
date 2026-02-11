# Assignment Workspace

This directory is mounted into the Docker container at `/home/rosuser/ros2_ws/src/assignment`.

## What to Place Here

Place your assignment files in this directory:

- **`gauntlet.world`** - Your custom Gazebo world file for the Digital Twin assignment
- **`gauntlet.yaml`** - The saved map file (YAML metadata)
- **`gauntlet.pgm`** - The saved map file (PGM image)
- Any other custom ROS 2 packages or files needed for your assignment

## Access from Container

Once inside the Docker container, you can access these files at:
```bash
/home/rosuser/ros2_ws/src/assignment/
```

## Example: Using a Custom World File

If you have a `gauntlet.world` file here, launch it with:
```bash
ros2 launch gazebo_ros gazebo.launch.py world:=/home/rosuser/ros2_ws/src/assignment/gauntlet.world
```

## Note

Files created or modified inside the container in this directory will persist on your host system, so you won't lose your work when the container stops.
