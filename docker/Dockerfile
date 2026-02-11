# ROS 2 Humble + Gazebo Classic 11 + TurtleBot3 Docker Environment
# Optimized for ARM64 (Asahi Linux on Apple Silicon)
# NOTE: First build takes 30-60 minutes due to building Gazebo from source

FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ============================================================================
# Stage 1: System dependencies
# ============================================================================

# System dependencies for building Gazebo Classic 11 from source
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    curl \
    lsb-release \
    gnupg \
    sudo \
    vim \
    nano \
    # X11/GUI support
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    x11-apps \
    # Gazebo Classic 11 build dependencies
    libboost-all-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libprotoc-dev \
    libtbb-dev \
    libfreeimage-dev \
    libgts-dev \
    libavformat-dev \
    libavcodec-dev \
    libswscale-dev \
    libtinyxml-dev \
    libtinyxml2-dev \
    libtar-dev \
    libcurl4-openssl-dev \
    libbullet-dev \
    libsimbody-dev \
    libhdf5-dev \
    libuuid-dev \
    libzip-dev \
    libjsoncpp-dev \
    libusb-1.0-0-dev \
    libgraphviz-dev \
    qtbase5-dev \
    libqwt-qt5-dev \
    libqt5opengl5-dev \
    libogre-1.9-dev \
    libeigen3-dev \
    uuid-dev \
    # Python/ROS tools
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    # ROS packages that ARE available on ARM64
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-teleop \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# Stage 2: Install Ignition/SDFormat dependencies
# ============================================================================

# Try to install SDFormat and Ignition libraries from repos
# Use || true to continue if not available on ARM64
RUN apt-get update && apt-get install -y \
    libsdformat12-dev \
    libignition-math6-dev \
    libignition-transport8-dev \
    libignition-common3-dev \
    libignition-fuel-tools4-dev \
    libignition-msgs5-dev \
    || true
RUN rm -rf /var/lib/apt/lists/*

# ============================================================================
# Stage 3: Build Gazebo Classic 11 from source
# ============================================================================

WORKDIR /tmp/gazebo_build

# Clone Gazebo Classic 11
RUN git clone --depth 1 --branch gazebo11 https://github.com/gazebosim/gazebo-classic.git

# Build and install
RUN cd gazebo-classic && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Clean up build files to reduce image size
RUN rm -rf /tmp/gazebo_build

# ============================================================================
# Stage 4: Build gazebo_ros_pkgs from source
# ============================================================================

WORKDIR /opt/gazebo_ros_ws/src

# Clone gazebo_ros_pkgs (humble branch)
RUN git clone --depth 1 --branch humble https://github.com/ros-simulation/gazebo_ros_pkgs.git

# Build with colcon
WORKDIR /opt/gazebo_ros_ws
RUN source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Clean up
RUN rm -rf /opt/gazebo_ros_ws/build /opt/gazebo_ros_ws/log

# ============================================================================
# Stage 5: Build TurtleBot3 simulation packages from source
# ============================================================================

WORKDIR /opt/tb3_sim_ws/src

# Clone TurtleBot3 simulations (humble branch)
RUN git clone --depth 1 --branch humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# Build
WORKDIR /opt/tb3_sim_ws
RUN source /opt/ros/humble/setup.bash && \
    source /opt/gazebo_ros_ws/install/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Clean up
RUN rm -rf /opt/tb3_sim_ws/build /opt/tb3_sim_ws/log

# ============================================================================
# Stage 6: User setup
# ============================================================================

# Initialize rosdep if not already done
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Create non-root user
RUN useradd -m -s /bin/bash -G sudo rosuser && \
    echo "rosuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Create workspace
RUN mkdir -p /home/rosuser/ros2_ws/src && \
    chown -R rosuser:rosuser /home/rosuser

# Environment variables
ENV TURTLEBOT3_MODEL=burger
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:/opt/tb3_sim_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models
ENV ROS_DOMAIN_ID=0

# Add ROS setup to bashrc for interactive shells
USER rosuser
RUN echo "source /opt/ros/humble/setup.bash" >> /home/rosuser/.bashrc && \
    echo "source /opt/gazebo_ros_ws/install/setup.bash" >> /home/rosuser/.bashrc && \
    echo "source /opt/tb3_sim_ws/install/setup.bash" >> /home/rosuser/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh 2>/dev/null || true" >> /home/rosuser/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /home/rosuser/.bashrc && \
    echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /home/rosuser/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:/opt/tb3_sim_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models:\$GAZEBO_MODEL_PATH" >> /home/rosuser/.bashrc

WORKDIR /home/rosuser/ros2_ws

COPY --chown=rosuser:rosuser entrypoint.sh /home/rosuser/entrypoint.sh
RUN chmod +x /home/rosuser/entrypoint.sh

ENTRYPOINT ["/home/rosuser/entrypoint.sh"]
CMD ["bash"]
