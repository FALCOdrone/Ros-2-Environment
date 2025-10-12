import numpy as np
import matplotlib.pyplot as plt


class DroneModel:
    def __init__(
            self,
            mass,
            inertia,  # Typical drone inertia values
            accel_variance,
            gyro_variance,
            dt):

        self.mass = mass
        self.inertia = inertia
        self.dt = dt
        # State: [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
        self.state = np.zeros(12)  
        self.clean_state = np.zeros(12)  # Clean state without sensor noise
        self.accel_variance = accel_variance
        self.gyro_variance = gyro_variance
        self.state_history = []
        self.clean_state_history = []
        self.noisy_state_history = []
        self.noisy_meas = []
        
        # Add small random initial offsets to noisy state to see divergence
        self.state += np.random.normal(0, 0.01, 12)  # Small initial noise

    def imu_model(self, accel_true, gyro_true):
        """
        Simulate IMU readings with noise and bias.
        
        In a real IMU:
        - Gyroscope measures angular velocities directly: ω_measured = ω_true + noise + bias
        - Accelerometer measures specific force (acceleration - gravity): a_measured = a_true + noise + bias
        
        Returns both clean and noisy sensor readings.
        """
        # Add white noise
        accel_noise = np.random.normal(0, self.accel_variance, 3)
        gyro_noise = np.random.normal(0, self.gyro_variance, 3)
        
        # Add small random bias that changes slowly over time (random walk)
        if not hasattr(self, '_accel_bias'):
            self._accel_bias = np.random.normal(0, 0.2, 3)  # Larger initial bias
            self._gyro_bias = np.random.normal(0, 0.1, 3)   # Larger initial bias
        
        # Slowly varying bias (random walk)
        self._accel_bias += np.random.normal(0, 0.005, 3)  # More bias drift
        self._gyro_bias += np.random.normal(0, 0.002, 3)   # More bias drift

        accel_noisy = accel_true + accel_noise + self._accel_bias
        gyro_noisy = gyro_true + gyro_noise + self._gyro_bias
        
        return (accel_true, gyro_true), (accel_noisy, gyro_noisy)

    def attitude_dynamics(self, torque, dt=None, torque_res=None):
        """
        Model of the drone's attitude dynamics.
        """
        if dt is None:
            dt = self.dt
        if torque_res is None:
            torque_res = np.zeros(3)
            
        # Current angular velocities in body frame
        omega_body = self.clean_state[9:12]
        
        # Skew-symmetric matrix for cross product
        omega_skew = np.array([[0, -omega_body[2], omega_body[1]],
                              [omega_body[2], 0, -omega_body[0]],
                              [-omega_body[1], omega_body[0], 0]])
        
        # Euler's equation for rigid body rotation
        omega_dot = np.linalg.inv(self.inertia) @ (
            torque + torque_res - omega_skew @ (self.inertia @ omega_body)
        )

        # Update clean angular velocities and orientation
        self.clean_state[9:12] += omega_dot * dt
        self.clean_state[6:9] += self.clean_state[9:12] * dt
        
        # Get noisy sensor readings for gyroscope
        (accel_clean, gyro_clean), (accel_noisy, gyro_noisy) = self.imu_model(
            np.array([0, 0, 0]),  # We'll compute proper acceleration in altitude_dynamics
            self.clean_state[9:12]
        )
        
        # Update noisy state using noisy gyro measurements
        # Integrate noisy angular velocities to get noisy orientation
        self.state[9:12] = gyro_noisy  # Store noisy angular velocities
        self.state[6:9] += self.state[9:12] * dt  # Integrate to get noisy orientation

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.
        
        Input:
            roll: Rotation around x-axis in radians.
            pitch: Rotation around y-axis in radians.
            yaw: Rotation around z-axis in radians.
        
        Output:
            Quaternion [w, x, y, z] format (scalar first).
        """
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return np.array([w, x, y, z])

    def quaternion_rot_matrix(self, state=None):
        """
        Convert quaternion to rotation matrix.
        """
        if state is None:
            state = self.clean_state
            
        roll, pitch, yaw = state[6], state[7], state[8]
        q = self.get_quaternion_from_euler(roll, pitch, yaw)
        w, x, y, z = q

        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
        ])

    def altitude_dynamics(self, thrust, dt=None, thrust_res=None):
        """
        Model of the drone's altitude dynamics.
        """
        if dt is None:
            dt = self.dt
        if thrust_res is None:
            thrust_res = np.zeros(3)
            
        # Current velocities
        velocity = self.clean_state[3:6]
        
        # Rotation matrix from body to world frame
        R = self.quaternion_rot_matrix()
        
        # Thrust in world frame (assuming thrust is in body Z direction)
        thrust_world = R @ np.array([0, 0, thrust])
        
        # Total force including gravity and external disturbances
        total_force = thrust_world + thrust_res + np.array([0, 0, -9.81 * self.mass])
        
        # Acceleration
        acceleration = total_force / self.mass
        
        # Update clean velocities and positions
        self.clean_state[3:6] += acceleration * dt
        self.clean_state[0:3] += self.clean_state[3:6] * dt

        # Get noisy sensor readings for acceleration
        (accel_clean, gyro_clean), (accel_noisy, gyro_noisy) = self.imu_model(
            acceleration, self.clean_state[9:12]
        )
        
        # Store the accelerometer measurement for sensor readings
        self._last_accel_measurement = accel_noisy.copy()
        
        # Update noisy state - integrate noisy accelerometer measurements
        # This shows the cumulative effect of accelerometer noise on velocity/position estimation
        self.state[3:6] += accel_noisy * dt  # Update noisy velocity using noisy acceleration
        self.state[0:3] += self.state[3:6] * dt  # Update noisy position using noisy velocity
        
        # Add orientation-dependent position errors (realistic coupling)
        # When drone is tilted, thrust creates lateral acceleration
        R_noisy = self.quaternion_rot_matrix(self.state)
        thrust_world_noisy = R_noisy @ np.array([0, 0, thrust])
        orientation_error_effect = (thrust_world_noisy[:2] - thrust_world[:2]) * dt * dt * 0.1
        self.state[0:2] += orientation_error_effect

        # Store history
        self.clean_state_history.append(self.clean_state.copy())
        self.noisy_state_history.append(self.state.copy())

    def plot_state(self):
        """
        Plot the state history comparing clean simulation vs noisy sensor data.
        """
        if not self.clean_state_history or not self.noisy_state_history:
            print("No state history available. Run simulation first.")
            return
            
        clean_array = np.array(self.clean_state_history)
        noisy_array = np.array(self.noisy_state_history)
        noisy_measurements = np.array(self.noisy_meas)
        noisy_gyro = np.array([m[1] for m in noisy_measurements])
        noisy_accel = np.array([m[0] for m in noisy_measurements])
        time = np.arange(clean_array.shape[0]) * self.dt

        plt.figure(figsize=(15, 12))

        # Position comparison
        plt.subplot(4, 1, 1)
        plt.plot(time, clean_array[:, 0], 'b-', label='X Clean', linewidth=0.5)
        plt.plot(time, noisy_array[:, 0], 'b--', label='X Noisy', alpha=1)
        plt.plot(time, clean_array[:, 1], 'g-', label='Y Clean', linewidth=0.5)
        plt.plot(time, noisy_array[:, 1], 'g--', label='Y Noisy', alpha=1)
        plt.plot(time, clean_array[:, 2], 'r-', label='Z Clean', linewidth=0.5)
        plt.plot(time, noisy_array[:, 2], 'r--', label='Z Noisy', alpha=1)
        plt.title('Position Comparison: Clean vs Noisy')
        plt.ylabel('Position (m)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Velocity comparison
        plt.subplot(4, 1, 2)
        plt.plot(time, clean_array[:, 3], 'b-', label='Vx Clean', linewidth=0.5)
        plt.plot(time, noisy_array[:, 3], 'b--', label='Vx Noisy', alpha=1)
        plt.plot(time, clean_array[:, 4], 'g-', label='Vy Clean', linewidth=1)
        plt.plot(time, noisy_array[:, 4], 'g--', label='Vy Noisy', alpha=1)
        plt.plot(time, clean_array[:, 5], 'r-', label='Vz Clean', linewidth=0.5)
        plt.plot(time, noisy_array[:, 5], 'r--', label='Vz Noisy', alpha=1)
        plt.title('Velocity Comparison: Clean vs Noisy')
        plt.ylabel('Velocity (m/s)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Orientation comparison
        plt.subplot(4, 1, 3)
        plt.plot(time, clean_array[:, 6], 'b-', label='Roll Clean', linewidth=0.5)
        plt.plot(time, noisy_array[:, 6], 'b--', label='Roll Noisy', alpha=1)
        plt.plot(time, clean_array[:, 7], 'g-', label='Pitch Clean', linewidth=0.5)
        plt.plot(time, noisy_array[:, 7], 'g--', label='Pitch Noisy', alpha=1)
        plt.plot(time, clean_array[:, 8], 'r-', label='Yaw Clean', linewidth=0.5)
        plt.plot(time, noisy_array[:, 8], 'r--', label='Yaw Noisy', alpha=1)
        plt.title('Orientation Comparison: Clean vs Noisy')
        plt.ylabel('Angle (rad)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Angular velocity comparison
        plt.subplot(4, 1, 4)
        plt.plot(time, clean_array[:, 9], 'b-', label='p Clean', linewidth=0.5)
        plt.plot(time, noisy_array[:, 9], 'b--', label='p Noisy', alpha=1)
        plt.plot(time, clean_array[:, 10], 'g-', label='q Clean', linewidth=0.5)
        plt.plot(time, noisy_array[:, 10], 'g--', label='q Noisy', alpha=1)
        plt.plot(time, clean_array[:, 11], 'r-', label='r Clean', linewidth=0.5)
        plt.plot(time, noisy_array[:, 11], 'r--', label='r Noisy', alpha=1)
        plt.title('Angular Velocity Comparison: Clean vs Noisy')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Gyroscope readings subplot
        plt.figure(figsize=(15, 4))
        plt.subplot(1, 1, 1)
        plt.plot(time, noisy_gyro[:, 0], 'b--', label='Gyro p Noisy', alpha=1)
        plt.plot(time, noisy_gyro[:, 1], 'g--', label='Gyro q Noisy', alpha=1)
        plt.plot(time, noisy_gyro[:, 2], 'r--', label='Gyro r Noisy', alpha=1)
        plt.title('Gyroscope Readings (Noisy)')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def get_state(self, noisy=True):
        """
        Get current state.
        
        Args:
            noisy: If True, return noisy state. If False, return clean state.
        """
        return self.state if noisy else self.clean_state
    
    def get_sensor_readings(self):
        """
        Get current sensor readings as they would appear from real IMU.
        
        Returns:
            dict: Contains 'gyro' (angular velocities) and 'accel' (accelerations)
                  These are the noisy measurements that a real IMU would provide.
        """
        return {
            'gyro': self.state[9:12].copy(),  # Direct gyro measurement (noisy angular velocities)
            'accel': getattr(self, '_last_accel_measurement', np.zeros(3))  # Last accel measurement
        }
    
    def get_clean_state(self):
        """Get clean simulation state without sensor noise."""
        return self.clean_state.copy()
    
    def get_noisy_state(self):
        """Get noisy state with sensor noise."""
        return self.state.copy()
    
    def reset(self):
        """Reset the drone model to initial conditions."""
        self.state = np.zeros(12)
        self.clean_state = np.zeros(12)
        self.state_history = []
        self.clean_state_history = []
        self.noisy_state_history = []
        self.noisy_meas = []
    
    def step(self, thrust, torque, dt=None):
        """
        Perform one simulation step with both attitude and altitude dynamics.
        
        Args:
            thrust: Thrust magnitude (scalar)
            torque: Torque vector [tx, ty, tz]
            dt: Time step (uses self.dt if None)
        """
        if dt is None:
            dt = self.dt
            
        # Update attitude first, then altitude
        self.attitude_dynamics(torque, dt)
        self.altitude_dynamics(thrust, dt)
        
        # Store measurement for plotting (get current sensor readings)
        sensor_data = self.get_sensor_readings()
        self.noisy_meas.append((sensor_data['accel'], sensor_data['gyro']))


# Real-time integration framework for control systems
class RealTimeDroneInterface:
    """
    Interface for integrating the drone model with real-time sensor data
    and control systems.
    """
    
    def __init__(self, drone_model, sensor_update_rate=100):
        self.drone_model = drone_model
        self.sensor_update_rate = sensor_update_rate  # Hz
        self.dt_sensor = 1.0 / sensor_update_rate
        self.last_sensor_time = 0
        self.control_commands = {'thrust': 0, 'torque': np.zeros(3)}
        
    def update_sensor_data(self, imu_data, timestamp):
        """
        Update with real IMU data.
        
        Args:
            imu_data: Dictionary with 'accel' and 'gyro' keys
            timestamp: Current timestamp
        """
        if timestamp - self.last_sensor_time >= self.dt_sensor:
            # Process real sensor data here
            # This would interface with actual hardware
            self.last_sensor_time = timestamp
            return True
        return False
    
    def set_control_command(self, thrust, torque):
        """Set control commands from controller."""
        self.control_commands['thrust'] = thrust
        self.control_commands['torque'] = np.array(torque)
    
    def run_simulation_step(self):
        """Run one simulation step with current control commands."""
        return self.drone_model.step(
            self.control_commands['thrust'],
            self.control_commands['torque']
        )
    
    def get_state_estimate(self):
        """
        Get state estimate (this is where sensor fusion would go).
        For now, returns the noisy sensor state.
        """
        return self.drone_model.get_noisy_state()


# Example usage and testing
def run_example_simulation():
    """Example simulation demonstrating the corrected model."""
    # Create drone model with realistic noise levels for IMU-only navigation
    drone = DroneModel(
        mass=1.0,
        inertia=np.diag([0.01, 0.01, 0.02]),  # Typical quadrotor inertia
        accel_variance=np.array([0.1, 0.1, 0.08]),  # Realistic accelerometer noise (m/s²)
        gyro_variance=np.array([0.05, 0.05, 0.03]),  # Realistic gyroscope noise (rad/s)
        dt=0.01
    )
    
    # Simulation parameters
    duration = 5.0  # seconds
    steps = int(duration / drone.dt)
    
    # Control inputs (simple hover test)
    hover_thrust = drone.mass * 9.81 + 5  # Compensate gravity

    print(f"Running simulation for {duration} seconds ({steps} steps)...")
    
    for i in range(steps):
        # More dynamic control inputs to better show noise effects
        thrust = hover_thrust + 2.0 * np.sin(i * drone.dt) + 1.0 * np.cos(i * drone.dt)
        torque = 0.01 * np.array([np.sin(i * drone.dt), 
                                np.cos(i * drone.dt), 
                                0.2 * np.sin(i * drone.dt)])
        
        drone.step(thrust, torque)
    
    print("Simulation complete!")
    print(f"Final clean position: {drone.get_clean_state()[:3]}")
    print(f"Final noisy position: {drone.get_noisy_state()[:3]}")
    
    # Plot results
    drone.plot_state()
    
    return drone


if __name__ == "__main__":
    # Run example simulation
    drone = run_example_simulation()