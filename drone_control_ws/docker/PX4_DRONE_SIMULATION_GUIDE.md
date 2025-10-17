# PX4 Gazebo Iris Drone Simulation Guide

Complete guide for running PX4 Iris drone simulation with Gazebo, MAVROS, and autonomous mission planning.

## 🎯 Overview

This setup provides a complete drone simulation environment with:
- **PX4 SITL**: Software-in-the-loop flight controller
- **Gazebo Classic**: 3D physics simulation with Iris quadcopter
- **MAVROS**: ROS2 ↔ PX4 communication bridge
- **Mission Commander**: Autonomous flight control node

## 🚀 Quick Start

### 1. Start the Docker Container

```bash
cd /path/to/Ros-2-Environment/drone_control_ws/docker
xhost + # activate the gui from x11
docker-compose up -d drone-control # run once
docker-compose exec drone-control bash
```

## 🔧 Manual Step-by-Step System launch

Starting components individually:

### Terminal 1: Start PX4 SITL with Gazebo
Start PX4 SITL, which simulates the flight controller:
```bash
cd /opt/px4_source
make px4_sitl gz_x500

# If you want to include the wind plugin, run the following
make px4_sitl gz_x500_wind 

# Alternative models available:
# make px4_sitl gazebo-classic_iris_irlock    # Iris with precision landing
# make px4_sitl gazebo-classic_typhoon_h480   # Typhoon hexacopter
# make px4_sitl gazebo-classic_plane          # Fixed-wing aircraft
```

### Terminal 2: Start MAVROS

Start MAVROS to bridge ROS2 and PX4:
```bash
cd /workspace
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:="udp://:14540@127.0.0.1:14580"
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

### Wind Scenarios for Testing

```bash
# Gentle breeze from west
pxh> param set SIM_WIND_V 3.0 && param set SIM_WIND_D 270 && param set SIM_WIND_T 0.5

# Strong crosswind from south
pxh> param set SIM_WIND_V 8.0 && param set SIM_WIND_D 180 && param set SIM_WIND_T 1.5

# Gusty conditions
pxh> param set SIM_WIND_V 6.0 && param set SIM_WIND_D 45 && param set SIM_WIND_T 3.0
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
- Verify MAVLink ports: Should see "udp port 14580 remote port 14540"
- Check MAVROS URL: `fcu_url:="udp://:14540@127.0.0.1:14580"`

**3. Gazebo GUI not showing:**
```bash
# On host machine:
xhost +local:docker
export DISPLAY=:0

# In container:
export DISPLAY=:0
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
```

## 🎯 Next Steps

1. **Modify Mission**: Edit `/workspace/src/drone_control/drone_control/mission_commander.py`
2. **Add Sensors**: Extend the simulation with camera, lidar, etc.
3. **Custom Controllers**: Implement your own control algorithms
4. **Wind Testing**: Use wind parameters to test robustness
5. **Formation Flight**: Launch multiple drones with different namespaces

## 📚 Additional Resources

- **PX4 Documentation**: https://docs.px4.io/
- **MAVROS Documentation**: https://github.com/mavlink/mavros
- **Gazebo Documentation**: http://gazebosim.org/
- **ROS2 Documentation**: https://docs.ros.org/en/humble/

---

**Success Indicators:**
- ✅ Gazebo shows Iris drone in 3D environment
- ✅ PX4 console shows `pxh>` prompt  
- ✅ MAVROS reports "FCU connection successful"
- ✅ Mission commander reports "OFFBOARD mode enabled"
- ✅ Drone autonomously takes off, flies pattern, and lands

Happy flying! 🚁
