# Base image
FROM ros:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
# Reduce compiler memory usage for emulation
ENV MAKEFLAGS="-j1"
ENV COLCON_PARALLEL_WORKERS=1

# Update and install OS dependencies
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    lsb-release \
    gnupg2 \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-gazebo-dev \
    ros-humble-control-toolbox \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create robotics user
RUN useradd -m -s /bin/bash robotics && \
    echo "robotics:robotics" | chpasswd && \
    adduser robotics sudo && \
    echo "robotics ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Switch to robotics user
USER robotics
WORKDIR /home/robotics

# Set up workspace
RUN mkdir -p /home/robotics/ros2_ws/src
WORKDIR /home/robotics/ros2_ws/src

# Clone the ros2_control_demos repository
RUN git clone https://github.com/ros-controls/ros2_control_demos.git -b humble

# Move back to root of workspace
WORKDIR /home/robotics/ros2_ws

# Source ROS setup and build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Set working directory
WORKDIR /home/robotics

# Source the workspace at container start
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /home/robotics/ros2_ws/install/setup.bash && exec bash"]