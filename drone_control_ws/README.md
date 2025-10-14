# ROS2 Drone Control and Simulation Environment

This ROS2 workspace provides a complete simulation environment for testing quadrotor drone control systems using Gazebo Ignition.

## Features

- **Physical Drone Model**: Realistic quadrotor with proper inertia and mass properties
- **Sensor Suite**: IMU, sonar (downward-facing), front camera, barometer, and magnetometer
- **Control System**: PID-based position and attitude control from your original implementation
- **State Estimation**: Basic sensor fusion for position and orientation estimation
- **Trajectory Planning**: Waypoint-based flight planning with obstacle avoidance
- **Gazebo Simulation**: Complete 3D environment with physics simulation

## Package Structure

```
drone_control_ws/
├── src/
│   ├── drone_description/     # URDF/Xacro drone model and sensors
│   ├── drone_control/         # Control system nodes (converted from your controllers)
│   ├── drone_sensors/         # Additional sensor simulation nodes
│   └── drone_simulation/      # Gazebo world and launch files
```

## System Architecture

### Topics
- `/drone/pose` - Estimated drone position and orientation
- `/drone/velocity` - Estimated drone velocities
- `/drone/setpoint` - Desired position and orientation
- `/drone/control_wrench` - Control commands (thrust and torques)
- `/drone/imu` - IMU sensor data
- `/drone/sonar` - Sonar altitude measurement
- `/drone/camera/image_raw` - Front camera images
- `/drone/barometer` - Barometric pressure
- `/drone/magnetometer` - Magnetic field measurements

### Nodes
- `drone_controller` - Main PID control loop
- `state_estimator` - Sensor fusion and state estimation
- `trajectory_planner` - Waypoint navigation
- `barometer_simulator` - Barometric pressure simulation
- `magnetometer_simulator` - Magnetic field simulation

## Installation and Setup

### Prerequisites
```bash
# Install ROS2 Humble (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop

# Install Gazebo Garden
sudo apt install ros-humble-gazebo-ros-pkgs

# Install additional dependencies
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-xacro
sudo apt install python3-scipy
```

### Build the Workspace
```bash
cd /home/lorenzo/Ros-2-Environment/drone_control_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Launch Complete Simulation
```bash
# Terminal 1: Launch the complete system
ros2 launch drone_simulation drone_complete.launch.py

# The system will start:
# - Gazebo with the drone world
# - Spawn the quadrotor drone
# - All control and sensor nodes
# - RViz for visualization
```

### Individual Components

#### Gazebo Only
```bash
ros2 launch drone_simulation gazebo.launch.py
```

#### Control System Only
```bash
ros2 launch drone_control control_system.launch.py
```

### Monitor System Status
```bash
# Check all running nodes
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /drone/pose
ros2 topic echo /drone/control_wrench

# View sensor data
ros2 topic echo /drone/imu
ros2 topic echo /drone/sonar
```

### Send Manual Commands
```bash
# Send position setpoint
ros2 topic pub /drone/setpoint geometry_msgs/PoseStamped "{
  header: {frame_id: 'world'},
  pose: {
    position: {x: 2.0, y: 1.0, z: -3.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

## Control System Details

The control system implements a cascaded PID controller based on your original implementation:

### Position Control (X, Y)
- Uses attitude (roll/pitch) as intermediate control
- PID gains: Kp=[2.0, 2.0], Ki=[1.1, 1.1], Kd=[1.0, 1.0]

### Altitude Control (Z)
- Direct thrust control with gravity compensation
- PID gains: Kp=3.0, Ki=0.05, Kd=1.5

### Attitude Control
- PID control for roll, pitch, yaw
- PID gains: Kp=[5.0, 5.0, 1.5], Ki=[1.1, 1.1, 0.05], Kd=[1.5, 1.0, 0.5]

### Safety Limits
- Maximum thrust: 42.183 N
- Maximum torque: 0.5 N⋅m
- Maximum angle: ±0.1 rad (±5.73°)
- Maximum acceleration: 0.5 m/s²

## Sensor Specifications

### IMU
- Update rate: 100 Hz
- Gaussian noise: σ = 0.01
- Bias drift simulation

### Sonar (Downward)
- Update rate: 20 Hz
- Range: 0.05 - 10.0 m
- Noise: σ = 0.02 m

### Camera (Front)
- Resolution: 640x480
- Update rate: 30 Hz
- FOV: 80° horizontal

### Barometer
- Update rate: 10 Hz
- Noise: σ = 0.1 hPa
- Altitude range: 0 - 10 km

### Magnetometer
- Update rate: 20 Hz
- Earth field: [0.2, 0.0, 0.4] Gauss (NED)
- Noise: σ = 0.01 Gauss

## Customization

### Modify Control Parameters
Edit `drone_control/drone_control/controllers.py` to adjust PID gains and limits.

### Add Waypoints
Modify `trajectory_planner.py` to add custom flight trajectories.

### Sensor Configuration
Update sensor properties in `drone_description/urdf/sensors.xacro`.

### World Environment
Edit `drone_simulation/worlds/drone_world.sdf` to add obstacles or change environment.

## Troubleshooting

### Common Issues

1. **Gazebo won't start**
   ```bash
   # Check Gazebo installation
   gazebo --version
   # Kill existing Gazebo processes
   killall gzserver gzclient
   ```

2. **Drone doesn't respond to commands**
   ```bash
   # Check if controller is running
   ros2 node list | grep drone_controller
   # Monitor control outputs
   ros2 topic echo /drone/control_wrench
   ```

3. **Sensors not publishing**
   ```bash
   # Check sensor nodes
   ros2 node list | grep -E "(barometer|magnetometer)"
   # Verify Gazebo plugins loaded
   ros2 topic list | grep drone
   ```

4. **Build errors**
   ```bash
   # Clean and rebuild
   cd drone_control_ws
   rm -rf build install log
   colcon build --symlink-install
   ```

## Development

### Testing Control System
The system includes a predefined square trajectory for testing:
1. Takeoff to 2m altitude
2. Fly square pattern (2m x 2m)
3. Return to origin
4. Land

### Extending the System
- Add new sensors by creating plugins in URDF
- Implement advanced control algorithms (LQR, MPC)
- Add obstacle avoidance and SLAM capabilities
- Integrate with external planning frameworks

## Performance Tips

- Use `--symlink-install` for faster development cycles
- Monitor system resources with `htop` during simulation
- Adjust Gazebo physics time step for performance vs accuracy
- Use RQT tools for real-time parameter tuning

## License

MIT License - Feel free to modify and extend for your research and development needs.
