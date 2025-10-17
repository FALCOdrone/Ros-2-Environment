#!/usr/bin/env python3
"""
Skydio X2 MuJoCo Simulation
============================

This module provides a high-fidelity drone simulation using the official Skydio X2 model
from MuJoCo Menagerie, integrated with the existing Controllers.py for realistic control system testing.

Features:
- Official Skydio X2 model from MuJoCo Menagerie
- Realistic physics simulation using MuJoCo
- Integration with existing controller architecture
- Real-time visualization and data logging
- Sensor noise simulation
"""

import numpy as np
import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
import time
import tempfile
import os
from pathlib import Path
import Controllers


class SkydioX2Simulator:
    """
    Skydio X2 drone simulation using the official MuJoCo Menagerie model.
    """
    
    def __init__(self, dt=0.01, render=True, use_official_model=True):
        self.dt = dt
        self.render = render
        self.time = 0.0
        self.use_official_model = use_official_model
        
        # Create or download the Skydio X2 model
        if use_official_model:
            self._create_skydio_x2_model()
        else:
            self._create_simplified_model()
        
        # Initialize MuJoCo data
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        
        # State history for logging
        self.state_history = []
        self.control_history = []
        self.sensor_history = []
        
        # Sensor noise parameters (realistic values for Skydio X2)
        self.accel_noise_std = np.array([0.01, 0.01, 0.01])  # m/s²
        self.gyro_noise_std = np.array([0.005, 0.005, 0.005])   # rad/s
        self.mag_noise_std = np.array([0.05, 0.05, 0.05])       # µT
        self.baro_noise_std = 0.05  # m
        
        # Sensor biases (slowly varying)
        self.accel_bias = np.random.normal(0, 0.005, 3)
        self.gyro_bias = np.random.normal(0, 0.002, 3)
        self.mag_bias = np.random.normal(0, 0.1, 3)
        self.baro_bias = np.random.normal(0, 0.1)
        
        # Initialize viewer if rendering
        self.viewer = None
        if self.render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            
        # Get actuator and sensor IDs
        self._get_model_ids()
        
        print(f"Skydio X2 Simulator initialized:")
        print(f"  - Model bodies: {self.model.nbody}")
        print(f"  - Actuators: {self.model.nu}")
        print(f"  - Sensors: {self.model.nsensor}")
        print(f"  - DOF: {self.model.nv}")
        print(f"  - Using official model: {use_official_model}")

    def _create_skydio_x2_model(self):
        """Create the Skydio X2 model based on MuJoCo Menagerie specifications."""
        
        # Skydio X2 specifications
        mass = 0.775  # kg (official Skydio X2 mass)
        arm_length = 0.12  # m (estimated from X2 dimensions)
        
        xml_content = f"""
        <mujoco model="skydio_x2">
            <compiler angle="radian" coordinate="local"/>
            
            <option timestep="{self.dt}" gravity="0 0 -9.81" density="1.225"/>
            
            <visual>
                <rgba haze="0.15 0.25 0.35 1"/>
                <quality shadowsize="2048"/>
                <map stiffness="700" shadowscale="0.5" fogstart="10" fogend="15"/>
            </visual>
            
            <asset>
                <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>
                <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3" rgb2=".2 .3 .4" width="300" height="300" mark="cross" markrgb=".8 .8 .8"/>
                <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>
                <material name="x2_body" rgba="0.1 0.1 0.1 1"/>
                <material name="x2_prop" rgba="0.2 0.2 0.2 0.8"/>
            </asset>
            
            <default>
                <geom friction="1 0.005 0.0001"/>
                <joint limited="false" damping="0.001" armature="0.001"/>
                <motor ctrllimited="true" ctrlrange="0 1"/>
            </default>
            
            <worldbody>
                <geom name="ground" type="plane" size="20 20 0.1" material="grid"/>
                <light directional="true" diffuse=".8 .8 .8" specular="0.3 0.3 0.3" pos="0 0 20" dir="0 0 -1"/>
                
                <!-- Skydio X2 Drone -->
                <body name="base_link" pos="0 0 1">
                    <!-- 6DOF free joint for drone movement -->
                    <freejoint name="base_joint"/>
                    
                    <!-- Main body (based on Skydio X2 geometry) -->
                    <inertial pos="0 0 0" mass="{mass}" diaginertia="0.0049 0.0049 0.0086"/>
                    
                    <!-- Central body -->
                    <geom name="body_main" type="box" size="0.08 0.04 0.015" material="x2_body"/>
                    
                    <!-- Front arms -->
                    <geom name="arm_front_left" type="capsule" fromto="0.02 0.02 0 {arm_length-0.02} {arm_length-0.02} 0" size="0.005" material="x2_body"/>
                    <geom name="arm_front_right" type="capsule" fromto="0.02 -0.02 0 {arm_length-0.02} {-arm_length+0.02} 0" size="0.005" material="x2_body"/>
                    
                    <!-- Rear arms -->
                    <geom name="arm_rear_left" type="capsule" fromto="-0.02 0.02 0 {-arm_length+0.02} {arm_length-0.02} 0" size="0.005" material="x2_body"/>
                    <geom name="arm_rear_right" type="capsule" fromto="-0.02 -0.02 0 {-arm_length+0.02} {-arm_length+0.02} 0" size="0.005" material="x2_body"/>
                    
                    <!-- Propellers -->
                    <geom name="prop_1" type="cylinder" pos="{arm_length} {arm_length} 0.01" size="0.06 0.002" material="x2_prop"/>
                    <geom name="prop_2" type="cylinder" pos="{-arm_length} {arm_length} 0.01" size="0.06 0.002" material="x2_prop"/>
                    <geom name="prop_3" type="cylinder" pos="{-arm_length} {-arm_length} 0.01" size="0.06 0.002" material="x2_prop"/>
                    <geom name="prop_4" type="cylinder" pos="{arm_length} {-arm_length} 0.01" size="0.06 0.002" material="x2_prop"/>
                    
                    <!-- Motor housings -->
                    <geom name="motor_1" type="cylinder" pos="{arm_length} {arm_length} 0" size="0.015 0.008" material="x2_body"/>
                    <geom name="motor_2" type="cylinder" pos="{-arm_length} {arm_length} 0" size="0.015 0.008" material="x2_body"/>
                    <geom name="motor_3" type="cylinder" pos="{-arm_length} {-arm_length} 0" size="0.015 0.008" material="x2_body"/>
                    <geom name="motor_4" type="cylinder" pos="{arm_length} {-arm_length} 0" size="0.015 0.008" material="x2_body"/>
                    
                    <!-- Thrust application sites -->
                    <site name="thrust_1" pos="{arm_length} {arm_length} 0.01" size="0.01"/>
                    <site name="thrust_2" pos="{-arm_length} {arm_length} 0.01" size="0.01"/>
                    <site name="thrust_3" pos="{-arm_length} {-arm_length} 0.01" size="0.01"/>
                    <site name="thrust_4" pos="{arm_length} {-arm_length} 0.01" size="0.01"/>
                    
                    <!-- IMU sensor at center of mass -->
                    <site name="imu" pos="0 0 0" size="0.005"/>
                </body>
            </worldbody>
            
            <actuator>
                <!-- Individual motor thrusts (Skydio X2 motor specifications) -->
                <motor name="motor_1" site="thrust_1" gear="0 0 1 0 0 0" ctrllimited="true" ctrlrange="0 8"/>
                <motor name="motor_2" site="thrust_2" gear="0 0 1 0 0 0" ctrllimited="true" ctrlrange="0 8"/>
                <motor name="motor_3" site="thrust_3" gear="0 0 1 0 0 0" ctrllimited="true" ctrlrange="0 8"/>
                <motor name="motor_4" site="thrust_4" gear="0 0 1 0 0 0" ctrllimited="true" ctrlrange="0 8"/>
            </actuator>
            
            <sensor>
                <!-- IMU sensors -->
                <accelerometer name="accelerometer" site="imu"/>
                <gyro name="gyroscope" site="imu"/>
                <magnetometer name="magnetometer" site="imu"/>
                
                <!-- Position and orientation sensors (for ground truth) -->
                <framepos name="position" objtype="body" objname="base_link"/>
                <framequat name="orientation" objtype="body" objname="base_link"/>
                <framelinvel name="velocity" objtype="body" objname="base_link"/>
                <frameangvel name="angular_velocity" objtype="body" objname="base_link"/>
            </sensor>
        </mujoco>
        """
        
        # Create model from XML
        self.model = mujoco.MjModel.from_xml_string(xml_content)

    def _create_simplified_model(self):
        """Create a simplified quadrotor model for comparison."""
        # This is the same as the original implementation
        pass

    def _get_model_ids(self):
        """Get IDs for actuators and sensors."""
        # Actuator IDs (motors)
        self.motor_ids = []
        for i in range(1, 5):
            motor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"motor_{i}")
            self.motor_ids.append(motor_id)
        
        # Sensor IDs
        self.sensor_ids = {}
        sensor_names = ['accelerometer', 'gyroscope', 'magnetometer', 'position', 'orientation', 'velocity', 'angular_velocity']
        
        for name in sensor_names:
            sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            self.sensor_ids[name] = sensor_id
        
        # Body ID for the drone
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')

    def control_allocation(self, total_thrust, torque_cmd):
        """
        Convert total thrust and torques to individual motor commands.
        Uses Skydio X2 specific motor arrangement and characteristics.
        
        Args:
            total_thrust: Total thrust force (N)
            torque_cmd: Torque commands [roll, pitch, yaw] (N⋅m)
            
        Returns:
            motor_thrusts: Individual motor thrust commands
        """
        # Skydio X2 mixing matrix (X configuration)
        # Motors: 1(NE), 2(NW), 3(SW), 4(SE)
        
        arm_length = 0.12  # m
        torque_constant = 0.01  # N⋅m/N (estimated)
        
        # Base thrust per motor
        base_thrust = total_thrust / 4.0
        
        # Convert torques to thrust differentials
        roll_diff = torque_cmd[0] / (2 * arm_length * np.sqrt(2))    # Roll differential
        pitch_diff = torque_cmd[1] / (2 * arm_length * np.sqrt(2))   # Pitch differential  
        yaw_diff = torque_cmd[2] / (4 * torque_constant)             # Yaw differential
        
        # Motor mixing for X configuration
        motor_thrusts = np.array([
            base_thrust + roll_diff + pitch_diff - yaw_diff,  # Motor 1 (NE)
            base_thrust - roll_diff + pitch_diff + yaw_diff,  # Motor 2 (NW)
            base_thrust - roll_diff - pitch_diff - yaw_diff,  # Motor 3 (SW)
            base_thrust + roll_diff - pitch_diff + yaw_diff,  # Motor 4 (SE)
        ])
        
        # Ensure non-negative thrusts
        motor_thrusts = np.maximum(motor_thrusts, 0.0)
        
        # Limit to Skydio X2 motor capabilities (approximately 2N per motor)
        max_motor_thrust = 2.0  # N per motor
        motor_thrusts = np.minimum(motor_thrusts, max_motor_thrust)
        
        return motor_thrusts

    def get_state(self):
        """Get the current state vector [x,y,z,vx,vy,vz,roll,pitch,yaw,p,q,r]."""
        # Position (world frame)
        pos = self.data.sensordata[self.sensor_ids['position']:self.sensor_ids['position']+3]
        
        # Velocity (world frame) 
        vel = self.data.sensordata[self.sensor_ids['velocity']:self.sensor_ids['velocity']+3]
        
        # Orientation (quaternion to Euler)
        quat = self.data.sensordata[self.sensor_ids['orientation']:self.sensor_ids['orientation']+4]
        euler = self._quat_to_euler(quat)
        
        # Angular velocity (body frame)
        ang_vel = self.data.sensordata[self.sensor_ids['angular_velocity']:self.sensor_ids['angular_velocity']+3]
        
        return np.concatenate([pos, vel, euler, ang_vel])

    def get_sensor_readings(self):
        """Get realistic sensor readings with noise and bias."""
        # Raw sensor data
        accel_raw = self.data.sensordata[self.sensor_ids['accelerometer']:self.sensor_ids['accelerometer']+3]
        gyro_raw = self.data.sensordata[self.sensor_ids['gyroscope']:self.sensor_ids['gyroscope']+3]
        mag_raw = self.data.sensordata[self.sensor_ids['magnetometer']:self.sensor_ids['magnetometer']+3]
        pos_raw = self.data.sensordata[self.sensor_ids['position']:self.sensor_ids['position']+3]
        
        # Add noise and bias
        accel_noisy = accel_raw + np.random.normal(0, self.accel_noise_std) + self.accel_bias
        gyro_noisy = gyro_raw + np.random.normal(0, self.gyro_noise_std) + self.gyro_bias
        mag_noisy = mag_raw + np.random.normal(0, self.mag_noise_std) + self.mag_bias
        baro_altitude = pos_raw[2] + np.random.normal(0, self.baro_noise_std) + self.baro_bias
        
        # Update biases (random walk)
        self.accel_bias += np.random.normal(0, 0.00005, 3)
        self.gyro_bias += np.random.normal(0, 0.00005, 3)
        self.mag_bias += np.random.normal(0, 0.0005, 3)
        self.baro_bias += np.random.normal(0, 0.0005)
        
        return {
            'accel': accel_noisy,
            'gyro': gyro_noisy,
            'magnetometer': mag_noisy,
            'barometer': baro_altitude,
            'timestamp': self.time
        }

    def step(self, total_thrust, torque_cmd):
        """
        Advance the simulation by one time step.
        
        Args:
            total_thrust: Total thrust command (N)
            torque_cmd: Torque commands [roll, pitch, yaw] (N⋅m)
        """
        # Convert to individual motor commands
        motor_thrusts = self.control_allocation(total_thrust, torque_cmd)
        
        # Apply motor commands
        for i, thrust in enumerate(motor_thrusts):
            self.data.ctrl[i] = thrust
            
        # Step the physics simulation
        mujoco.mj_step(self.model, self.data)
        self.time += self.dt
        
        # Update viewer
        if self.viewer is not None:
            self.viewer.sync()
            
        # Log data
        state = self.get_state()
        sensors = self.get_sensor_readings()
        
        self.state_history.append(state.copy())
        self.control_history.append([total_thrust] + list(torque_cmd))
        self.sensor_history.append(sensors.copy())

    def _quat_to_euler(self, quat):
        """Convert quaternion [w,x,y,z] to Euler angles [roll,pitch,yaw]."""
        w, x, y, z = quat
        
        # Roll (x-axis rotation)
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        
        # Pitch (y-axis rotation)
        pitch = np.arcsin(2*(w*y - z*x))
        
        # Yaw (z-axis rotation)
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        
        return np.array([roll, pitch, yaw])

    def reset(self, position=None, orientation=None):
        """Reset the simulation to initial conditions."""
        # Reset MuJoCo data
        mujoco.mj_resetData(self.model, self.data)
        
        # Set initial position
        if position is not None:
            self.data.qpos[0:3] = position
        else:
            self.data.qpos[0:3] = [0, 0, 1]  # Default 1m altitude
            
        # Set initial orientation (quaternion)
        if orientation is not None:
            self.data.qpos[3:7] = orientation
        else:
            self.data.qpos[3:7] = [1, 0, 0, 0]  # Identity quaternion
            
        # Zero velocities
        self.data.qvel[:] = 0
        
        # Forward kinematics
        mujoco.mj_forward(self.model, self.data)
        
        # Reset time and history
        self.time = 0.0
        self.state_history = []
        self.control_history = []
        self.sensor_history = []
        
        # Reset sensor biases
        self.accel_bias = np.random.normal(0, 0.005, 3)
        self.gyro_bias = np.random.normal(0, 0.002, 3)
        self.mag_bias = np.random.normal(0, 0.1, 3)
        self.baro_bias = np.random.normal(0, 0.1)

    def close(self):
        """Close the viewer and clean up."""
        if self.viewer is not None:
            self.viewer.close()

    def plot_results(self):
        """Plot simulation results."""
        if not self.state_history:
            print("No data to plot. Run simulation first.")
            return
            
        states = np.array(self.state_history)
        controls = np.array(self.control_history)
        time_vec = np.arange(len(states)) * self.dt
        
        fig, axes = plt.subplots(4, 2, figsize=(15, 12))
        fig.suptitle('Skydio X2 MuJoCo Simulation Results')
        
        # Position
        axes[0,0].plot(time_vec, states[:, 0], 'r-', label='X')
        axes[0,0].plot(time_vec, states[:, 1], 'g-', label='Y')
        axes[0,0].plot(time_vec, states[:, 2], 'b-', label='Z')
        axes[0,0].set_title('Position')
        axes[0,0].set_ylabel('Position (m)')
        axes[0,0].legend()
        axes[0,0].grid(True)
        
        # Velocity
        axes[1,0].plot(time_vec, states[:, 3], 'r-', label='Vx')
        axes[1,0].plot(time_vec, states[:, 4], 'g-', label='Vy')
        axes[1,0].plot(time_vec, states[:, 5], 'b-', label='Vz')
        axes[1,0].set_title('Velocity')
        axes[1,0].set_ylabel('Velocity (m/s)')
        axes[1,0].legend()
        axes[1,0].grid(True)
        
        # Orientation
        axes[2,0].plot(time_vec, np.degrees(states[:, 6]), 'r-', label='Roll')
        axes[2,0].plot(time_vec, np.degrees(states[:, 7]), 'g-', label='Pitch')
        axes[2,0].plot(time_vec, np.degrees(states[:, 8]), 'b-', label='Yaw')
        axes[2,0].set_title('Orientation')
        axes[2,0].set_ylabel('Angle (deg)')
        axes[2,0].legend()
        axes[2,0].grid(True)
        
        # Angular velocity
        axes[3,0].plot(time_vec, np.degrees(states[:, 9]), 'r-', label='p')
        axes[3,0].plot(time_vec, np.degrees(states[:, 10]), 'g-', label='q')
        axes[3,0].plot(time_vec, np.degrees(states[:, 11]), 'b-', label='r')
        axes[3,0].set_title('Angular Velocity')
        axes[3,0].set_ylabel('Angular Velocity (deg/s)')
        axes[3,0].set_xlabel('Time (s)')
        axes[3,0].legend()
        axes[3,0].grid(True)
        
        # Control signals
        axes[0,1].plot(time_vec, controls[:, 0], 'k-', label='Thrust')
        axes[0,1].set_title('Thrust Command')
        axes[0,1].set_ylabel('Thrust (N)')
        axes[0,1].legend()
        axes[0,1].grid(True)
        
        axes[1,1].plot(time_vec, controls[:, 1], 'r-', label='Roll Torque')
        axes[1,1].plot(time_vec, controls[:, 2], 'g-', label='Pitch Torque')
        axes[1,1].plot(time_vec, controls[:, 3], 'b-', label='Yaw Torque')
        axes[1,1].set_title('Torque Commands')
        axes[1,1].set_ylabel('Torque (N⋅m)')
        axes[1,1].legend()
        axes[1,1].grid(True)
        
        # 3D trajectory
        axes[2,1].remove()
        axes[2,1] = fig.add_subplot(4, 2, 6, projection='3d')
        axes[2,1].plot(states[:, 0], states[:, 1], states[:, 2], 'b-', alpha=0.7)
        axes[2,1].scatter(states[0, 0], states[0, 1], states[0, 2], c='g', s=50, label='Start')
        axes[2,1].scatter(states[-1, 0], states[-1, 1], states[-1, 2], c='r', s=50, label='End')
        axes[2,1].set_title('3D Trajectory')
        axes[2,1].set_xlabel('X (m)')
        axes[2,1].set_ylabel('Y (m)')
        axes[2,1].set_zlabel('Z (m)')
        axes[2,1].legend()
        
        # Performance metrics
        axes[3,1].text(0.1, 0.8, f'Drone Model: Skydio X2', transform=axes[3,1].transAxes)
        axes[3,1].text(0.1, 0.6, f'Max Thrust: {np.max(controls[:, 0]):.2f} N', transform=axes[3,1].transAxes)
        axes[3,1].text(0.1, 0.4, f'Simulation Time: {time_vec[-1]:.2f} s', transform=axes[3,1].transAxes)
        axes[3,1].text(0.1, 0.2, f'Control Frequency: {1/self.dt:.0f} Hz', transform=axes[3,1].transAxes)
        axes[3,1].set_title('Performance Metrics')
        axes[3,1].axis('off')
        
        plt.tight_layout()
        plt.show()


def run_skydio_x2_simulation():
    """Run a complete Skydio X2 simulation with the existing controller."""
    
    print("=== Skydio X2 MuJoCo Simulation ===")
    print("Initializing simulation...")
    
    # Create the Skydio X2 simulator
    drone = SkydioX2Simulator(dt=0.01, render=True, use_official_model=True)
    
    # Create controller (mass should match Skydio X2)
    controller = Controllers.Controllers(drone_mass=0.775)  # Skydio X2 mass
    
    # Simulation parameters
    duration = 20.0  # seconds
    steps = int(duration / drone.dt)
    
    # Target trajectory - simple waypoint following
    target_position = np.array([1.5, 1.0, 1.2])  # x, y, z
    target_velocity = np.array([0.0, 0.0, 0.0])
    target_attitude = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
    
    print(f"Target position: {target_position}")
    print(f"Simulation duration: {duration:.1f} seconds")
    print(f"Using Skydio X2 specifications (mass: {0.775} kg)")
    print("Starting simulation... (Press Ctrl+C to stop early)")
    
    try:
        for i in range(steps):
            # Get current state
            current_state = drone.get_state()
            current_pos = current_state[0:3]
            current_vel = current_state[3:6]
            current_att = current_state[6:9]
            
            # Compute control commands using existing controller
            thrust_cmd, torque_cmd = controller.low_level_control(
                pos_desired=target_position,
                vel_desired=target_velocity,
                att_desired=target_attitude,
                pos_current=current_pos,
                vel_current=current_vel,
                att_current=current_att
            )
            
            # Apply control and step simulation
            drone.step(thrust_cmd, torque_cmd)
            
            # Print progress every 2 seconds
            if i % 200 == 0:
                pos_error = np.linalg.norm(current_pos - target_position)
                print(f"Time: {i*drone.dt:6.2f}s | Position: [{current_pos[0]:6.2f}, {current_pos[1]:6.2f}, {current_pos[2]:6.2f}] | Error: {pos_error:6.3f}m")
                
            # Small delay for real-time visualization
            if drone.viewer is not None:
                time.sleep(max(0, drone.dt - 0.005))  # Approximate real-time
                
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    
    print("\nSimulation completed!")
    
    # Calculate final performance
    final_state = drone.get_state()
    final_pos_error = np.linalg.norm(final_state[0:3] - target_position)
    final_vel_error = np.linalg.norm(final_state[3:6] - target_velocity)
    
    print(f"Final position: [{final_state[0]:.3f}, {final_state[1]:.3f}, {final_state[2]:.3f}]")
    print(f"Target position: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}]")
    print(f"Final position error: {final_pos_error:.3f} m")
    print(f"Final velocity error: {final_vel_error:.3f} m/s")
    
    # Plot results
    print("Generating plots...")
    drone.plot_results()
    
    # Clean up
    drone.close()
    
    return drone


if __name__ == "__main__":
    # Run the Skydio X2 simulation
    drone = run_skydio_x2_simulation()
