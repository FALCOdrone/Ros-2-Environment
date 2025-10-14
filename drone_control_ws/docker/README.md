# ROS2 Humble + Gazebo Ignition Docker Environment

This Docker setup provides a complete environment for testing the drone control system with ROS2 Humble and Gazebo Ignition Garden.

## Prerequisites

- Docker installed and running
- X11 server (for GUI applications)
  - Linux: Usually pre-installed
  - macOS: Install XQuartz
  - Windows: Install VcXsrv or similar

## Quick Start

### 1. Build the Docker Image

```bash
cd drone_control_ws
./docker/build.sh
```

This will create a Docker image with:
- ROS2 Humble Desktop Full
- Gazebo Ignition Garden
- All necessary dependencies
- Your drone control packages

### 2. Run the Container

```bash
./docker/start.sh
```

This will start an interactive container with:
- GUI support for Gazebo and RViz
- Your workspace mounted at `/workspace`
- All ROS2 packages built and ready

### 3. Test the Setup

Inside the container, you can:

```bash
# Source the environment (already done in bashrc)
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# Launch the complete simulation
ros2 launch drone_simulation drone_simulation.launch.py

# Or launch individual components
ros2 launch drone_description display.launch.py  # View drone in RViz
ros2 run drone_control drone_controller_node     # Run controller
ros2 run drone_sensors sensor_simulator          # Run sensor nodes
```

## Alternative: Using Docker Compose

You can also use docker-compose:

```bash
# Build and run
docker-compose up --build

# Run in background
docker-compose up -d

# Stop
docker-compose down
```

## Package Structure

```
drone_control_ws/
├── src/
│   ├── drone_description/     # URDF models and robot description
│   ├── drone_control/         # Control algorithms and nodes
│   ├── drone_sensors/         # Sensor simulation nodes
│   └── drone_simulation/      # Gazebo worlds and launch files
├── docker/
│   ├── Dockerfile            # Main Docker image definition
│   ├── build.sh             # Build script
│   ├── start.sh             # Run script
│   └── docker-compose.yml   # Docker Compose configuration
└── README.md
```

## Available ROS2 Topics

### Control Topics
- `/drone/cmd_position` - Position commands (geometry_msgs/PoseStamped)
- `/drone/cmd_velocity` - Velocity commands (geometry_msgs/Twist)
- `/drone/state` - Current drone state (nav_msgs/Odometry)

### Sensor Topics
- `/drone/imu` - IMU data (sensor_msgs/Imu)
- `/drone/sonar` - Sonar distance (sensor_msgs/Range)
- `/drone/camera/image_raw` - Camera feed (sensor_msgs/Image)
- `/drone/barometer` - Pressure/altitude (sensor_msgs/FluidPressure)
- `/drone/magnetometer` - Magnetic field (sensor_msgs/MagneticField)

### Control Outputs
- `/drone/thrust` - Thrust command (std_msgs/Float64)
- `/drone/torque` - Torque commands (geometry_msgs/Vector3)

## Troubleshooting

### GUI Applications Not Working

**Linux:**
```bash
xhost +local:docker
```

**macOS:**
- Install XQuartz
- Enable "Allow connections from network clients" in XQuartz preferences
- Restart XQuartz

**Windows:**
- Install VcXsrv or similar X11 server
- Configure to allow connections

### Container Won't Start

1. Check Docker is running: `docker info`
2. Rebuild image: `./docker/build.sh`
3. Check logs: `docker logs drone_control_container`

### ROS2 Packages Not Found

Inside container:
```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

### Gazebo Performance Issues

1. Reduce simulation complexity in world files
2. Lower sensor update rates
3. Use headless mode: `gz sim -s` (no GUI)

## Development Workflow

1. **Edit code** on host system using your favorite editor
2. **Build** inside container: `cd /workspace && colcon build`
3. **Test** with: `ros2 launch drone_simulation drone_simulation.launch.py`
4. **Debug** with RViz: `rviz2`

## Configuration

### Environment Variables
- `ROS_DOMAIN_ID=0` - ROS2 domain
- `GAZEBO_MODEL_PATH` - Additional model paths
- `DISPLAY` - X11 display for GUI

### Volumes
- `../:/workspace` - Your workspace
- `/tmp/.X11-unix` - X11 socket for GUI
- `/dev` - Device access for hardware

## Performance Tips

1. **Build optimization**: Use `--symlink-install` for faster rebuilds
2. **Parallel builds**: `colcon build --parallel-workers 4`
3. **Selective building**: `colcon build --packages-select drone_control`
4. **Debug builds**: `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug`

## Advanced Usage

### Custom Worlds

Create custom Gazebo worlds in `src/drone_simulation/worlds/`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="custom_world">
    <!-- Your world definition -->
  </world>
</sdf>
```

### Additional Sensors

Add new sensors in `src/drone_description/urdf/sensors.xacro` and corresponding ROS2 nodes in `src/drone_sensors/`.

### Custom Controllers

Extend the control system in `src/drone_control/drone_control/` with additional control algorithms.
