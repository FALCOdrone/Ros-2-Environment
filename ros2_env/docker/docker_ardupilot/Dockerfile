# Dockerfile for ArduPilot simulation environment with ROS2 and MAVROS
FROM ubuntu:22.04

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies only
RUN apt update && \
    apt install -y git python3-pip python3-dev python3-opencv sudo \
                   curl wget && \
    pip install MAVProxy && \
    apt clean && rm -rf /var/lib/apt/lists/*

# Create a non-root user for ArduPilot installation
RUN useradd -m -s /bin/bash ardupilot && \
    echo "ardupilot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    usermod -a -G dialout ardupilot

# Switch to the ardupilot user
USER ardupilot
WORKDIR /home/ardupilot

# Clone ArduPilot repository as the ardupilot user (only if not exists)
RUN if [ ! -d "ardupilot" ]; then \
        # Configure git for better network handling \
        git config --global http.postBuffer 1048576000 && \
        git config --global http.maxRequestBuffer 100M && \
        git config --global core.compression 0 && \
        # Try multiple times with different approaches \
        git clone --depth 1 https://github.com/ArduPilot/ardupilot.git || \
        (sleep 10 && git clone --depth 1 https://github.com/ArduPilot/ardupilot.git) || \
        (sleep 20 && git clone https://github.com/ArduPilot/ardupilot.git); \
    fi

# Change to ardupilot directory and update submodules with better error handling
WORKDIR /home/ardupilot/ardupilot
RUN if [ ! -f ".gitmodules_updated" ]; then \
        git config --global http.postBuffer 1048576000 && \
        git config --global http.maxRequestBuffer 100M && \
        git config --global core.compression 0 && \
        (git submodule update --init --recursive --jobs 1 --depth 1 || \
         (sleep 30 && git submodule update --init --recursive --jobs 1 --depth 1) || \
         (sleep 60 && git submodule update --init --recursive --jobs 1)) && \
        touch .gitmodules_updated; \
    fi

# Install build prerequisites as non-root user (modify the script to skip usermod)
RUN sed -i 's/sudo usermod -a -G dialout.*/echo "User already in dialout group"/' Tools/environment_install/install-prereqs-ubuntu.sh && \
    sed -i 's/usermod -a -G dialout.*/echo "User already in dialout group"/' Tools/environment_install/install-prereqs-ubuntu.sh && \
    Tools/environment_install/install-prereqs-ubuntu.sh -y && \
    echo "source ~/.profile" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the working directory to ArduCopter
WORKDIR /home/ardupilot/ardupilot/ArduCopter

# Expose any necessary ports (adjust as needed)
EXPOSE 5760 5761 5762 5763

# Default command to run the simulation
CMD ["/bin/bash", "-c", "source ~/.profile && sim_vehicle.py -v ArduCopter --console --map"]
