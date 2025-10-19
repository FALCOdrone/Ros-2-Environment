# PX4 Gazebo x500 Drone Simulation Guide

Complete guide for running PX4 x500 drone simulation with Gazebo (gz), MAVROS, and autonomous mission planning.

## 🎯 Overview

This setup provides a complete drone simulation environment with:
- **PX4 SITL**: Software-in-the-loop flight controller
- **Gazebo (gz)**: Modern 3D physics simulation with x500 quadcopter
- **MAVROS**: ROS2 ↔ PX4 communication bridge
- **Mission Commander**: Autonomous flight control node

## 🚀 Quick Start

### 1. Start the Docker Container

```bash
cd /path/to/Ros-2-Environment/drone_control_ws/docker

# Before building the container make sure that X11 server is running on the host
xhost +

# Build the compose file
docker-compose build drone-control

# Launch the container
docker-compose up -d drone-control
docker-compose exec drone-control bash
```

### 2. One-Command Launch

Inside the container, use the integrated launch script:
```bash
# Launch PX4 SITL with Gazebo x500 model
/usr/local/bin/launch_px4_gz_x500.sh

# Or for full system (PX4 + Gazebo + MAVROS)
/usr/local/bin/start_integrated_system.sh
```

### 3. Available Helper Scripts

The container includes several helper scripts for different scenarios:

```bash
# Launch PX4 SITL with x500 model
/usr/local/bin/launch_px4_gz_x500.sh

# Launch only Gazebo simulation (no PX4)
/usr/local/bin/launch_gz_only.sh

# Launch only MAVROS
/usr/local/bin/launch_mavros.sh

# Launch integrated system (PX4 + Gazebo + MAVROS)
/usr/local/bin/start_integrated_system.sh
```

## 🔧 Manual Step-by-Step System launch

Starting components individually:

### Terminal 1: Start PX4 SITL with Gazebo
Start PX4 SITL, which simulates the flight controller:
```bash
cd /opt/px4_source
make px4_sitl gz_x500

# Alternative models available:
# make px4_sitl gz_x500_depth     # x500 with depth camera
# make px4_sitl gz_advanced_plane # Fixed-wing aircraft
# make px4_sitl gz_standard_vtol  # VTOL aircraft
```

### Terminal 2: Start MAVROS

Start MAVROS to bridge ROS2 and PX4:
```bash
cd /workspace
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

# Alternative: Use the helper script
/usr/local/bin/launch_mavros.sh
```

### Terminal 3: Start Mission Commander
```bash
cd /workspace
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build
source install/setup.bash

# Run the mission commander through the executable
./install/drone_control/bin/mission_commander
```

## Launching a custom control system and/or SLAM package without px4 and mavros
You can also launch Gazebo with the x500 model alone to test your own control algorithms:

```bash
cd /workspace

# (optional) run the setup script if you want to use ROS2 packages
source /opt/ros/humble/setup.bash

# Build the workspace first
cd /workspace
colcon build
source install/setup.bash

cd /workspace/docker
./launch_gazebo_x500_auto.sh
```
Therefore, from ros nodes you can publish thrust commands directly to the motor topics, implement your own flight controller as ROS 2 nodes. You can use `/gz/msgs/Double` for motor commands that needs to be sent to gazebo.
In addition, if you want to add wind disturbances to the simulation, you can set forces and torques in real-time while using gazebo and while the drone is flying. You will see three dots in the top right corner of the gazebo window, then you will find forces and torques options to set the wind disturbances.

## 🎮 Mission Control Commands

### View Available ROS2 Topics
```bash
ros2 topic list
```

**Key Topics:**
- `/mavros/state` - Vehicle connection and mode status
- `/mavros/local_position/pose` - Current drone position
- `/mavros/setpoint_position/local` - Target position commands
- `/mavros/cmd/arming` - Arm/disarm commands

### Monitor Drone Status
```bash
# Watch connection status
ros2 topic echo /mavros/state

# Monitor drone position
ros2 topic echo /mavros/local_position/pose

# View mission commands being sent
ros2 topic echo /mavros/setpoint_position/local
```

### Manual Control Commands
```bash
# Arm the drone
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Change flight mode to OFFBOARD
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"

# Emergency land
ros2 service call /mavros/cmd/land mavros_msgs/srv/CommandTOL "{}"
```

## 🌪️ Adding Wind Disturbances

### Real-time Wind Injection via PX4 Console

In the PX4 terminal (pxh> prompt):

```bash
# Add 5 m/s wind from the east with turbulence
pxh> param set SIM_WIND_V 5.0      # Wind speed (m/s)
pxh> param set SIM_WIND_D 90       # Wind direction (degrees: 0=N, 90=E, 180=S, 270=W)
pxh> param set SIM_WIND_T 2.0      # Turbulence intensity (0-10)

# Verify parameters
pxh> param show SIM_WIND_V
pxh> param show SIM_WIND_D
pxh> param show SIM_WIND_T

# Save parameters (optional)
pxh> param save

# Remove wind
pxh> param set SIM_WIND_V 0.0
```


## 🏁 Mission Commander Behavior

The autonomous mission commander executes:

1. **Connection Phase**: Waits for MAVROS connection to PX4
2. **Mode Setting**: Switches PX4 to OFFBOARD mode
3. **Arming**: Arms the vehicle for flight
4. **Takeoff**: Climbs to 3.0m altitude
5. **Square Pattern**: Flies a 10m x 10m square pattern
6. **Landing**: Returns to takeoff point and lands
7. **Disarm**: Safely disarms the vehicle

### x500 Model Characteristics

The x500 quadcopter offers several advantages over the previous Iris model:
- **Modern Design**: Updated aerodynamics and sensor suite
- **Better Stability**: Improved flight characteristics in windy conditions
- **Enhanced Sensors**: More realistic IMU, GPS, and barometer simulation
- **Gazebo Integration**: Native support for modern Gazebo physics
- **Modular Design**: Easy to add cameras, lidar, and other payloads

### Mission Waypoints (Square Pattern)
```python
# Default waypoints (can be modified in mission_commander.py)
waypoints = [
    [0.0, 0.0, 3.0],    # Takeoff position
    [10.0, 0.0, 3.0],   # Point 1
    [10.0, 10.0, 3.0],  # Point 2  
    [0.0, 10.0, 3.0],   # Point 3
    [0.0, 0.0, 3.0],    # Return to start
    [0.0, 0.0, 0.5]     # Landing position
]
```

## 🔧 Troubleshooting

### Common Issues

**1. "No executable found" error:**
```bash
cd /workspace
colcon build --packages-select drone_control
source install/setup.bash
```

**2. MAVROS not connecting:**
- Check PX4 is running: Look for `pxh>` prompt
- Verify MAVLink ports: Should see "udp port 14557 remote port 14540"
- Check MAVROS URL: `fcu_url:="udp://:14540@127.0.0.1:14557"`

**3. Gazebo GUI not showing:**
```bash
# On host machine:
xhost +local:docker
export DISPLAY=:0

# In container:
export DISPLAY=:0

# For Gazebo (gz) specific issues:
export GZ_SIM_RESOURCE_PATH=/opt/px4_source/Tools/simulation/gz/models:/opt/px4_source/Tools/simulation/gz/worlds
```

**4. Mission commander stuck on "Waiting for FCU connection":**
- Ensure PX4 SITL is running first
- Start MAVROS and wait for "FCU connection successful"
- Then start mission commander

### Verify System Status

**Check PX4 Status:**
```bash
# In PX4 console (pxh>):
pxh> commander status
pxh> mavlink status
```

**Check Gazebo Status:**
```bash
# List Gazebo topics
gz topic -l

# Check if x500 model is loaded
gz model -l
```

**Check MAVROS Connection:**
```bash
ros2 topic echo /mavros/state --once
# Should show: connected: true
```

**List Active Nodes:**
```bash
ros2 node list
# Should include: /mavros, /mission_commander
```

## 📊 Performance Monitoring

### Flight Data Logging

PX4 automatically logs flight data:
```bash
# View log files
ls /opt/px4_source/log/$(date +%Y-%m-%d)/

# Analyze logs (if ulog tools installed)
ulog_info /opt/px4_source/log/$(date +%Y-%m-%d)/*.ulg
```

### Real-time Monitoring

```bash
# Position accuracy
ros2 topic echo /mavros/local_position/pose

# Control effort  
ros2 topic echo /mavros/actuator_outputs

# System diagnostics
ros2 topic echo /mavros/diagnostics

# Gazebo-specific monitoring
gz topic -e -t /world/default/pose/info
```

## 🎯 Next Steps

1. **Modify Mission**: Edit `/workspace/src/drone_control/drone_control/mission_commander.py`
2. **Add Sensors**: Extend the simulation with camera, lidar, etc.
3. **Custom Controllers**: Implement your own control algorithms
4. **Wind Testing**: Use wind parameters to test robustness
5. **Formation Flight**: Launch multiple drones with different namespaces
6. **Advanced Models**: Try x500_depth for computer vision applications

## 📚 Additional Resources

- **PX4 Documentation**: https://docs.px4.io/
- **PX4 Gazebo Integration**: https://docs.px4.io/main/en/sim_gazebo_gz/
- **MAVROS Documentation**: https://github.com/mavlink/mavros
- **Gazebo Documentation**: https://gazebosim.org/
- **ROS2 Documentation**: https://docs.ros.org/en/humble/

---

**Success Indicators:**
- ✅ Gazebo shows x500 drone in 3D environment
- ✅ PX4 console shows `pxh>` prompt  
- ✅ MAVROS reports "FCU connection successful"
- ✅ Mission commander reports "OFFBOARD mode enabled"
- ✅ Drone autonomously takes off, flies pattern, and lands

Happy flying! 🚁
