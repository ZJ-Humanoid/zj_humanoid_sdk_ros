# Dockerfile for zj_humanoid_mock ROS package
# Based on ROS Noetic

FROM ros:noetic-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/catkin_ws

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    python3-rospy \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    && rm -rf /var/lib/apt/lists/*

# Copy and install zj_humanoid_types package
# Note: Build context should be set to project root directory
COPY api_struct/zj_humanoid_types_25_R3.run /tmp/zj_humanoid_types_25_R3.run
RUN chmod +x /tmp/zj_humanoid_types_25_R3.run && \
    cd /tmp && \
    # Run installer non-interactively
    # The installer extracts .deb files and installs them via dpkg
    # Use printf to provide 'y' responses for any prompts
    printf 'y\ny\ny\n' | /tmp/zj_humanoid_types_25_R3.run 2>&1 || \
    # If interactive prompt fails, try with yes command
    (yes | /tmp/zj_humanoid_types_25_R3.run 2>&1 || true) && \
    # Verify installation by checking if ROS can find the packages
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rospack find audio >/dev/null 2>&1 || echo "Warning: Some message packages may not be installed" && \
    # Clean up installer
    rm -f /tmp/zj_humanoid_types_25_R3.run && \
    # Clean up any temporary extraction directories (but keep installed packages)
    find /tmp -maxdepth 1 -type d -name "zj_humanoid_types*" -exec rm -rf {} + 2>/dev/null || true

# Create catkin workspace
RUN mkdir -p ${CATKIN_WS}/src

# Copy mock package to workspace
COPY mock_packages/zj_humanoid_mock/ ${CATKIN_WS}/src/zj_humanoid_mock/

# Build the workspace
WORKDIR ${CATKIN_WS}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make"

# Copy entrypoint script
COPY mock_packages/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Source the workspace in bashrc
RUN echo "source ${CATKIN_WS}/devel/setup.bash" >> ~/.bashrc

# Set working directory
WORKDIR ${CATKIN_WS}

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command: run mock server
CMD ["mock"]

