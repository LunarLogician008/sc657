# ROS 2 Humble + Gazebo Classic + TurtleBot3 Docker Environment

Docker-based development environment for ROS 2 Humble with Gazebo Classic 11 and TurtleBot3, optimized for **Asahi Ubuntu 24.04 on Apple Silicon (ARM64)**.

## Features

- **ROS 2 Humble** (native ARM64 support, no emulation)
- **Gazebo Classic 11** (not Gz Sim)
- **TurtleBot3** packages (Burger model)
- **Nav2** navigation stack
- **SLAM Toolbox** and **Cartographer** for mapping
- **RViz2** for visualization
- **Full GUI support** via X11 forwarding
- **Host networking** for seamless ROS 2 DDS communication

## Prerequisites

- Docker and Docker Compose installed
- X11 server running (default on Ubuntu)
- Asahi Ubuntu 24.04 on MacBook M2 (or any ARM64 Linux system)

## Quick Start

```bash
cd docker
chmod +x run.sh
./run.sh
```

This will:
1. Build the Docker image (first time only, may take 10-15 minutes)
2. Set up X11 access for GUI applications
3. Start the container with an interactive bash shell

## Opening Additional Terminals

Once the container is running, open additional terminal windows in the container:

```bash
docker exec -it ros2_humble_turtlebot3 bash
```

You can run different ROS 2 commands in each terminal (e.g., Gazebo in one, SLAM in another, RViz in a third).

---

## Assignment Week 1: Digital Twin Commands

### Step 1: Launch Gazebo with TurtleBot3

**Option A: Use the default TurtleBot3 world**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Option B: Use a custom world file (e.g., gauntlet.world)**

First, place your `gauntlet.world` file in the `workspace/` directory, then:
```bash
ros2 launch gazebo_ros gazebo.launch.py world:=/home/rosuser/ros2_ws/src/assignment/gauntlet.world
```

You should see the Gazebo GUI open with the TurtleBot3 spawned in the world.

---

### Step 2: Launch SLAM for Mapping

Open a **new terminal** inside the container:
```bash
docker exec -it ros2_humble_turtlebot3 bash
```

Then start SLAM Toolbox:
```bash
ros2 launch slam_toolbox online_async_launch.py
```

Alternatively, use Cartographer SLAM (if preferred):
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

---

### Step 3: Launch RViz2 for Visualization

Open **another new terminal** inside the container:
```bash
docker exec -it ros2_humble_turtlebot3 bash
```

Launch RViz2:
```bash
ros2 launch nav2_bringup rviz_launch.py
```

Or launch RViz2 manually and load a custom config:
```bash
rviz2
```

In RViz2, you should see:
- The TurtleBot3 robot model
- Laser scan data
- Map being built in real-time (from SLAM)

---

### Step 4: Teleoperate the Robot to Build the Map

Open **another new terminal** inside the container:
```bash
docker exec -it ros2_humble_turtlebot3 bash
```

Run the teleop keyboard node:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i` - move forward
- `,` - move backward
- `j` - turn left
- `l` - turn right
- `k` - stop
- `q/z` - increase/decrease max speeds
- `w/x` - increase/decrease only linear speed
- `e/c` - increase/decrease only angular speed

Drive the robot around the environment to map the entire area.

---

### Step 5: Save the Map

Once you've mapped the entire environment, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/rosuser/ros2_ws/src/assignment/gauntlet
```

This creates two files in the `workspace/` directory:
- `gauntlet.yaml` - Map metadata (resolution, origin, etc.)
- `gauntlet.pgm` - Map image (occupancy grid)

These files persist on your host system in `docker/workspace/`.

---

### Step 6: Launch Nav2 with the Saved Map

After saving the map, you can use it for autonomous navigation.

**Stop SLAM** (Ctrl+C in the SLAM terminal), then launch the Nav2 map server:

```bash
ros2 launch nav2_bringup map_server.launch.py map:=/home/rosuser/ros2_ws/src/assignment/gauntlet.yaml
```

For full Nav2 navigation stack:
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True map:=/home/rosuser/ros2_ws/src/assignment/gauntlet.yaml
```

---

## Configuration Space (C-Space) and Obstacle Inflation

### TurtleBot3 Burger Dimensions

The **TurtleBot3 Burger** is a circular robot with:
- **Radius**: ~0.105 m (105 mm)
- **Footprint**: Circular, centered at the robot's base

### Minkowski Sum and C-Space

The **Configuration Space (C-Space)** represents all possible positions the robot's center can occupy without collision.

To compute C-Space:
1. **Shrink the robot to a point** (its center)
2. **Inflate obstacles by the robot's radius**

This is called the **Minkowski Sum** of the workspace and the robot's geometry.

**Example:**
- If the robot has radius `r = 0.105 m`
- A wall in the world needs to be inflated by `0.105 m` outward
- The robot's center must stay at least `0.105 m` away from any wall

### Nav2 Costmap Inflation

Nav2 uses **costmap layers** to represent obstacle inflation:

**Key parameters** (in `costmap_params.yaml` or similar):

```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0
  inflation_radius: 0.55  # Should be > robot radius for safety margin
```

- **`inflation_radius`**: How far to inflate obstacles (in meters)
  - Set to `robot_radius + safety_margin`
  - Example: `0.105 + 0.1 = 0.205 m` minimum, but often set to ~0.5m for smoother paths

- **`cost_scaling_factor`**: How quickly cost decreases with distance from obstacle
  - Higher values = sharper cost gradient

**Recommended starting values for TurtleBot3 Burger:**
- `inflation_radius: 0.3` to `0.55`
- `cost_scaling_factor: 3.0`

To visualize inflation in RViz2:
1. Add a "Map" display
2. Set topic to `/global_costmap/costmap` or `/local_costmap/costmap`
3. You'll see obstacles (black), inflated zones (gradient), and free space (white)

---

## Asahi-Specific Troubleshooting

### Software Rendering Fallback

The Asahi GPU drivers may not fully support hardware acceleration inside Docker containers.

**Solution:** The environment variable `LIBGL_ALWAYS_SOFTWARE=1` is set by default to use software rendering (Mesa LLVMpipe). This ensures Gazebo and RViz2 work but may be slower.

### Gazebo GUI Performance Issues

If the Gazebo GUI is slow or unresponsive:

**Option 1: Use headless Gazebo + RViz2**
```bash
# Terminal 1: Run Gazebo server only (no GUI)
gzserver /opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world

# Terminal 2: Spawn TurtleBot3
ros2 run gazebo_ros spawn_entity.py -entity turtlebot3_burger -file /opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf

# Terminal 3: Use RViz2 for visualization
rviz2
```

**Option 2: Reduce Gazebo GUI quality**

Edit Gazebo settings (`~/.gazebo/gui.ini`) or launch with minimal rendering.

### X11 Forwarding Issues

**Problem:** GUI applications don't display or show "cannot open display" error.

**Solutions:**

1. **Check `DISPLAY` variable:**
   ```bash
   echo $DISPLAY
   ```
   Should show something like `:0` or `:1`.

2. **Allow Docker X11 access:**
   ```bash
   xhost +local:docker
   ```

3. **Verify X11 socket mount:**
   Ensure `docker-compose.yml` has:
   ```yaml
   volumes:
     - /tmp/.X11-unix:/tmp/.X11-unix:rw
   environment:
     - DISPLAY=${DISPLAY}
   ```

4. **Test X11 forwarding:**
   Inside container:
   ```bash
   xeyes
   ```
   You should see a window with eyes following your cursor.

### ARM64 Package Availability

Most ROS 2 Humble packages have native ARM64 builds. If you encounter a missing package:

1. **Search for ARM64 availability:**
   ```bash
   apt-cache search ros-humble-<package-name>
   ```

2. **Build from source if needed:**
   ```bash
   cd ~/ros2_ws/src
   git clone <package-repo>
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

### GPU Device Access

The `docker-compose.yml` mounts `/dev/dri` for GPU access. On Asahi, GPU support is still evolving.

If you get permission errors:
```bash
sudo chmod 666 /dev/dri/renderD128
```

Or add your user to the `render` group:
```bash
sudo usermod -aG render $USER
```
Then log out and back in.

---

## Container Management

### Start Container
```bash
./run.sh
```

### Stop Container
Press `Ctrl+C` in the terminal running `./run.sh`, or:
```bash
docker compose down
```

### Rebuild Image After Changes
```bash
docker compose build --no-cache
```

### Remove Everything and Start Fresh
```bash
docker compose down -v
docker rmi ros2-humble-turtlebot3:latest
./run.sh
```

---

## Environment Variables Reference

| Variable | Value | Purpose |
|----------|-------|---------|
| `TURTLEBOT3_MODEL` | `burger` | Selects TurtleBot3 Burger model |
| `LIBGL_ALWAYS_SOFTWARE` | `1` | Forces software rendering for GPU compatibility |
| `GAZEBO_MODEL_PATH` | `/opt/ros/humble/share/turtlebot3_gazebo/models` | Path to TurtleBot3 models |
| `DISPLAY` | `:0` or `:1` | X11 display for GUI apps |

---

## File Structure

```
docker/
├── Dockerfile              # Docker image definition
├── docker-compose.yml      # Container orchestration
├── entrypoint.sh           # Container startup script
├── run.sh                  # Convenience script to start container
├── workspace/              # Mounted into container at /home/rosuser/ros2_ws/src/assignment
│   └── README.md           # Workspace usage instructions
└── README.md               # This file
```

---

## Additional Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Gazebo Classic Documentation](https://classic.gazebosim.org/)

---

## Support

If you encounter issues:

1. Check the **Asahi-Specific Troubleshooting** section above
2. Verify Docker and Docker Compose are installed: `docker --version && docker compose version`
3. Ensure X11 is running: `echo $DISPLAY`
4. Check container logs: `docker logs ros2_humble_turtlebot3`

**For assignment-specific questions, contact your course instructor.**
