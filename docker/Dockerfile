# ROS 2 Humble + Gazebo Classic 11 + TurtleBot3 Docker Environment
# Optimized for ARM64 (Asahi Linux on Apple Silicon)

FROM ros:humble

# Set environment to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo Classic 11, TurtleBot3, Nav2, SLAM, and GUI tools
RUN apt-get update && apt-get install -y \
    # Gazebo Classic 11
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    gazebo \
    # TurtleBot3 packages
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-teleop \
    # Nav2 packages
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    # SLAM packages
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    # Visualization & Teleop
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    # Build tools
    python3-colcon-common-extensions \
    python3-rosdep \
    # GUI/X11 support
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    x11-apps \
    # Utilities
    sudo \
    vim \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep if not already done
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Create a non-root user with sudo access
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up workspace
RUN mkdir -p /home/$USERNAME/ros2_ws/src && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

# Switch to non-root user
USER $USERNAME
WORKDIR /home/$USERNAME

# Set environment variables
ENV TURTLEBOT3_MODEL=burger
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH

# Add ROS setup to bashrc for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh 2>/dev/null || true" >> /home/$USERNAME/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /home/$USERNAME/.bashrc && \
    echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /home/$USERNAME/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:\$GAZEBO_MODEL_PATH" >> /home/$USERNAME/.bashrc

# Copy entrypoint script
COPY --chown=$USERNAME:$USERNAME entrypoint.sh /home/$USERNAME/entrypoint.sh
RUN chmod +x /home/$USERNAME/entrypoint.sh

WORKDIR /home/$USERNAME/ros2_ws

ENTRYPOINT ["/home/rosuser/entrypoint.sh"]
CMD ["bash"]
