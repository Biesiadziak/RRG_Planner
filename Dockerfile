# Use the official ROS 2 Jazzy base image
FROM osrf/ros:jazzy-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Update and install required packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    git \
    wget \
    curl \
    nano \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb*
    
# Initialize rosdep
RUN rosdep update

# Source ROS 2 in every new shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Set working directory for your ROS 2 workspace
WORKDIR /root/ros2_ws

# Default command: open a shell
CMD ["/bin/bash"]
