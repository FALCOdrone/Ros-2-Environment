import numpy as np
import matplotlib.pyplot as plt
import Controllers

class LinearizedDroneModel:
    def __init__(
            self,
            mass,
            inertia,
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
        
        # Gravity constant
        self.g = 9.81

    def imu_model(self, accel_true, gyro_true):
        """
        Simulate IMU readings with noise and bias.
        """
        # Add white noise
        accel_noise = np.random.normal(0, self.accel_variance, 3)
        gyro_noise = np.random.normal(0, self.gyro_variance, 3)
        
        # Add small random bias that changes slowly over time (random walk)
        if not hasattr(self, '_accel_bias'):
            self._accel_bias = np.random.normal(0, 0.02, 3)  # Smaller initial bias for stability
            self._gyro_bias = np.random.normal(0, 0.01, 3)   # Smaller initial bias for stability
        
        # Slowly varying bias (random walk) - reduced for stability
        self._accel_bias += np.random.normal(0, 0.001, 3)  # Less bias drift
        self._gyro_bias += np.random.normal(0, 0.0005, 3)   # Less bias drift

        accel_noisy = accel_true + accel_noise + self._accel_bias
        gyro_noisy = gyro_true + gyro_noise + self._gyro_bias
        
        return (accel_true, gyro_true), (accel_noisy, gyro_noisy)

    def linearized_dynamics(self, thrust, torque, dt=None):
        """
        Linearized drone dynamics around hover condition.
        
        Assumptions:
        - Small angles: sin(θ) ≈ θ, cos(θ) ≈ 1
        - Hover condition: thrust ≈ mg
        - Decoupled dynamics for position and attitude
        """
        if dt is None:
            dt = self.dt
            
        # Current state variables
        x, y, z = self.clean_state[0:3]
        vx, vy, vz = self.clean_state[3:6]
        roll, pitch, yaw = self.clean_state[6:9]
        p, q, r = self.clean_state[9:12]
        
        # === LINEARIZED POSITION DYNAMICS ===
        # For small angles, the thrust creates acceleration approximately:
        # ax ≈ (thrust/mass) * pitch
        # ay ≈ -(thrust/mass) * roll  
        # az ≈ (thrust/mass) - g
        
        ax = (thrust / self.mass) * pitch
        ay = -(thrust / self.mass) * roll
        az = (thrust / self.mass) - self.g
        
        # Update velocities
        self.clean_state[3] += ax * dt  # vx
        self.clean_state[4] += ay * dt  # vy
        self.clean_state[5] += az * dt  # vz
        
        # Update positions
        self.clean_state[0] += vx * dt  # x
        self.clean_state[1] += vy * dt  # y
        self.clean_state[2] += vz * dt  # z
        
        # === LINEARIZED ATTITUDE DYNAMICS ===
        # Simplified angular acceleration (decoupled)
        # For small angles, the inertia matrix becomes approximately diagonal
        Ixx, Iyy, Izz = self.inertia[0,0], self.inertia[1,1], self.inertia[2,2]
        
        p_dot = torque[0] / Ixx
        q_dot = torque[1] / Iyy  
        r_dot = torque[2] / Izz
        
        # Update angular velocities
        self.clean_state[9] += p_dot * dt   # p
        self.clean_state[10] += q_dot * dt  # q
        self.clean_state[11] += r_dot * dt  # r
        
        # Update angles (small angle assumption: angular velocity ≈ angle rate)
        self.clean_state[6] += p * dt   # roll
        self.clean_state[7] += q * dt   # pitch
        self.clean_state[8] += r * dt   # yaw
        
        # === SENSOR SIMULATION ===
        # Get noisy sensor readings
        true_accel = np.array([ax, ay, az])
        true_gyro = np.array([p, q, r])
        
        (accel_clean, gyro_clean), (accel_noisy, gyro_noisy) = self.imu_model(true_accel, true_gyro)
        
        # Store the accelerometer measurement for sensor readings
        self._last_accel_measurement = accel_noisy.copy()
        
        # Update noisy state using noisy sensor measurements
        # Position estimation using noisy accelerometer
        self.state[3:6] += accel_noisy * dt  # noisy velocity
        self.state[0:3] += self.state[3:6] * dt  # noisy position
        
        # Attitude estimation using noisy gyroscope
        self.state[9:12] = gyro_noisy  # noisy angular velocities
        self.state[6:9] += self.state[9:12] * dt  # noisy orientation
        
        # Store history
        self.clean_state_history.append(self.clean_state.copy())
        self.noisy_state_history.append(self.state.copy())

    def step(self, thrust, torque, dt=None):
        """
        Perform one simulation step with linearized dynamics.
        """
        if dt is None:
            dt = self.dt
            
        # Use linearized dynamics
        self.linearized_dynamics(thrust, torque, dt)
        
        # Store measurement for plotting
        sensor_data = self.get_sensor_readings()
        self.noisy_meas.append((sensor_data['accel'], sensor_data['gyro']))

    def get_state(self, noisy=True):
        """Get current state."""
        return self.state if noisy else self.clean_state
    
    def get_sensor_readings(self):
        """Get current sensor readings as they would appear from real IMU."""
        return {
            'gyro': self.state[9:12].copy(),
            'accel': getattr(self, '_last_accel_measurement', np.zeros(3))
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

    def plot_state(self):
        """Plot the state history comparing clean simulation vs noisy sensor data."""
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
        plt.plot(time, noisy_array[:, 0], 'b-', label='X Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 1], 'g-', label='Y Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 2], 'r-', label='Z Noisy', alpha=1)
        plt.title('Position Comparison: Clean vs Noisy (Linearized Model)')
        plt.ylabel('Position (m)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Velocity comparison
        plt.subplot(4, 1, 2)
        plt.plot(time, noisy_array[:, 3], 'b-', label='Vx Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 4], 'g-', label='Vy Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 5], 'r-', label='Vz Noisy', alpha=1)
        plt.title('Velocity Comparison: Clean vs Noisy (Linearized Model)')
        plt.ylabel('Velocity (m/s)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Orientation comparison
        plt.subplot(4, 1, 3)
        plt.plot(time, noisy_array[:, 6], 'b-', label='Roll Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 7], 'g-', label='Pitch Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 8], 'r-', label='Yaw Noisy', alpha=1)
        plt.title('Orientation Comparison: Clean vs Noisy (Linearized Model)')
        plt.ylabel('Angle (rad)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Angular velocity comparison
        plt.subplot(4, 1, 4)
        plt.plot(time, noisy_array[:, 9], 'b-', label='p Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 10], 'g-', label='q Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 11], 'r-', label='r Noisy', alpha=1)
        plt.title('Angular Velocity Comparison: Clean vs Noisy (Linearized Model)')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()


# Alias for compatibility
DroneModel = LinearizedDroneModel


def run_example_simulation():
    """Example simulation demonstrating the linearized model."""
    # Drone parameters
    drone_mass = 1.0  # kg
    
    # Create drone model with realistic noise levels for IMU-only navigation
    drone = LinearizedDroneModel(
        mass=drone_mass,
        inertia=np.diag([0.01, 0.01, 0.02]),  # Typical quadrotor inertia
        accel_variance=np.array([0.1, 0.1, 0.08]),  # Realistic accelerometer noise (m/s²)
        gyro_variance=np.array([0.05, 0.05, 0.03]),  # Realistic gyroscope noise (rad/s)
        dt=0.01
    )
    
    # Set initial position closer to target to reduce initial error
    drone.clean_state[2] = 0.5  # Start at 0.5m height instead of 0.1m
    drone.state[2] = 0.5
    
    # Simulation parameters
    duration = 8.0  # seconds (shorter for faster testing)
    steps = int(duration / drone.dt)
    
    # Set the reference altitude and attitude
    ref_xyz = np.array([0.0, 0.0, 1.0])  # Desired position (hover at 1m)
    ref_attitude = np.zeros(3)  # Roll, pitch, yaw
    ref_angular_velocity = np.zeros(3)  # p, q, r
    ref_lin_velocity = np.zeros(3)  # vx, vy, vz

    # Create controller instance
    controller = Controllers.Controllers(drone_mass=drone_mass)

    print(f"Running linearized simulation for {duration} seconds ({steps} steps)...")
    print(f"Drone mass: {drone_mass} kg")
    print(f"Gravity compensation: {drone_mass * 9.81:.2f} N")
    
    for i in range(steps):
        # Get current state
        current_state = drone.get_noisy_state()
        
        # Let's compute the low level control inputs to maintain hover
        forceZ_cmd, torque_cmd = controller.low_level_control(
            pos_desired=ref_xyz,
            vel_desired=ref_lin_velocity,
            att_desired=ref_attitude,
            pos_current=current_state[:3],
            vel_current=current_state[3:6],
            att_current=current_state[6:9]
        )
        
        # Apply control commands
        thrust = forceZ_cmd
        torque = np.array(torque_cmd)

        # Step the simulation
        drone.step(thrust, torque)
        
        # Print progress every second
        if i % 100 == 0:
            clean_pos = drone.get_clean_state()[:3]
            z_error = ref_xyz[2] - clean_pos[2]
            print(f"t={i*drone.dt:.1f}s: Clean pos=[{clean_pos[0]:.3f}, {clean_pos[1]:.3f}, {clean_pos[2]:.3f}], Thrust={thrust:.2f}N, Z_error={z_error:.3f}m")
    
    print("Simulation complete!")
    print(f"Final clean position: {drone.get_clean_state()[:3]}")
    print(f"Final noisy position: {drone.get_noisy_state()[:3]}")
    print(f"Target position: {ref_xyz}")
    
    # Calculate final errors
    final_clean_error = np.linalg.norm(drone.get_clean_state()[:3] - ref_xyz)
    final_noisy_error = np.linalg.norm(drone.get_noisy_state()[:3] - ref_xyz)
    print(f"Final clean position error: {final_clean_error:.3f} m")
    print(f"Final noisy position error: {final_noisy_error:.3f} m")
    
    # Plot results
    drone.plot_state()
    
    return drone


if __name__ == "__main__":
    # Run example simulation
    drone = run_example_simulation()
