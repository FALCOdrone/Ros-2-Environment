# Falco Drone Control Package

This package provides a controllable quadrotor drone simulation for ROS2 and Gazebo Ignition.

## Overview

This package has been updated to provide proper drone control capabilities. You now have multiple options for controlling the drone:

1. **Simple Force-based Control** (Recommended for testing)
2. **Hector-style Controller** (For compatibility with existing ROS packages)
3. **Modern Multirotor Controller** (For advanced simulation)

## New Files Added

### URDF Models:
- `urdf/quadrotor_base_with_motors.urdf.xacro` - Quadrotor with motor links/joints
- `urdf/quadrotor_ignition_controller.urdf.xacro` - Modern Gazebo controller
- `urdf/quadrotor_simple_force_controller.urdf.xacro` - Simple force-based control
- `urdf/quadrotor_simple_test.urdf.xacro` - Minimal working test model

### Launch Files:
- `launch/gazebo_controllable.launch.py` - Full-featured controllable quadrotor
- `launch/gazebo_simple_controllable.launch.py` - Simple force-based control
- `launch/teleop.launch.py` - Keyboard control interface

### Scripts:
- `scripts/teleop_quadrotor.py` - Keyboard teleop control
- `scripts/cmd_vel_to_wrench.py` - Converts cmd_vel to force/torque commands

### Modified Files:
- `package.xml` - Added necessary dependencies
- `urdf/sensors/quadrotor_sensors.urdf.xacro` - Updated for Gazebo Ignition
- `CMakeLists.txt` - Added new scripts

## Installation & Setup

### 1. Build the package:

```bash
cd /home/lorenzo/Ros-2-Environment/ros2_env/ros2_ws
colcon build --packages-select falco_drone
source install/setup.bash
```

### 2. Install dependencies (if needed):

```bash
sudo apt install ros-$ROS_DISTRO-ros-gz-bridge ros-$ROS_DISTRO-ros-gz-sim
```

## Usage Options

### Option 1: Simple Force-based Control (Recommended)

This is the most reliable method for getting started:

```bash
# Terminal 1: Launch simulation
cd /home/lorenzo/Ros-2-Environment/ros2_env/ros2_ws
source install/setup.bash
ros2 launch falco_drone gazebo_simple_controllable.launch.py

# Terminal 2: Launch teleop control
cd /home/lorenzo/Ros-2-Environment/ros2_env/ros2_ws
source install/setup.bash
ros2 run falco_drone teleop_quadrotor.py
```

### Option 2: Full-featured Controllable Quadrotor

```bash
# Terminal 1: Launch simulation
cd /home/lorenzo/Ros-2-Environment/ros2_env/ros2_ws
source install/setup.bash
ros2 launch falco_drone gazebo_controllable.launch.py

# Terminal 2: Launch teleop control
cd /home/lorenzo/Ros-2-Environment/ros2_env/ros2_ws
source install/setup.bash
ros2 run falco_drone teleop_quadrotor.py
```

### Option 3: Manual Command Control

You can also control the drone directly via command line:

```bash
# Takeoff (move up)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Move forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Turn right
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}'

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## Keyboard Controls

When using the teleop script:

```
Control Your Quadrotor!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i - Move forward
, - Move backward  
j - Turn left
l - Turn right
u - Move forward + turn left
o - Move forward + turn right
m - Move backward + turn left
. - Move backward + turn right
t - Move up
b - Move down

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease linear speed by 10%  
e/c : increase/decrease angular speed by 10%
space key, k : force stop
CTRL-C to quit
```

## Topics

The drone uses the following ROS2 topics:

- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/model/quadrotor_0/link/base_link/wrench` (geometry_msgs/Wrench) - Force/torque commands
- `/quadrotor_0/imu` (sensor_msgs/Imu) - IMU data (when available)
- `/quadrotor_0/ground_truth/state` (nav_msgs/Odometry) - Position data (when available)

## Troubleshooting

### Problem: Drone doesn't respond to commands

**Solution 1**: Check if topics are active:
```bash
ros2 topic list
ros2 topic echo /cmd_vel
```

**Solution 2**: Try the simple force-based controller:
```bash
ros2 launch falco_drone gazebo_simple_controllable.launch.py
```

**Solution 3**: Manually publish commands:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {z: 1.0}}'
```

### Problem: Gazebo won't start

**Solution**: Check your Gazebo installation:
```bash
gz sim --version
echo $GZ_SIM_RESOURCE_PATH
```

### Problem: Scripts not executable

**Solution**: Make scripts executable:
```bash
chmod +x /home/lorenzo/Ros-2-Environment/ros2_env/ros2_ws/src/falco_drone/scripts/*.py
```

## Configuration

You can modify control parameters in:

- `scripts/cmd_vel_to_wrench.py` - Force scaling factors
- `urdf/quadrotor_*_controller.urdf.xacro` - Controller gains and limits

### Force Scaling (in cmd_vel_to_wrench.py):
```python
self.force_scale = 10.0      # Scale factor for linear forces
self.torque_scale = 5.0      # Scale factor for angular torques  
self.hover_force = 14.5      # Force needed to hover
```

## Next Steps

1. Start with the simple force-based control to verify basic functionality
2. Test keyboard control with the teleop script
3. If working, try the full-featured controller
4. Customize force/torque scaling for your desired responsiveness
5. Add additional sensors or control features as needed

## Support

If you encounter issues:
1. Check that all dependencies are installed
2. Verify the build completed without errors
3. Try the simple test model first
4. Check ROS2 topic communication with `ros2 topic list` and `ros2 topic echo`
