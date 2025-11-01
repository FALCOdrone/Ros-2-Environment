import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
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
        self.filtered_sensor = np.zeros(6)  # 3 accel + 3 gyro
        self.alpha = 0.95  # Increased filter strength for better noise reduction
        
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

        # Flatten the IMU measurements for filtering
        imu_meas = np.concatenate([accel_noisy, gyro_noisy])

        # apply a low pass filter for the imu measurments
        self.filtered_sensor = (1 - self.alpha) * self.filtered_sensor + imu_meas * self.alpha
        
        return (accel_true, gyro_true), (accel_noisy, gyro_noisy)
    
    def compute_derivatives(self, state, thrust, torque):
        """
        Compute the time derivatives of the linearized system to use them for Runge_Kutta integration
        given the current state, thrust, and torque.
        """
        x, y, z = state[0:3]
        vx, vy, vz = state[3:6]
        roll, pitch, yaw = state[6:9]
        p, q, r = state[9:12]

        ax = (thrust / self.mass) * pitch
        ay = -(thrust / self.mass) * roll
        az = (thrust / self.mass) - self.g

        Ixx, Iyy, Izz = self.inertia[0,0], self.inertia[1,1], self.inertia[2,2]
        p_dot = torque[0] / Ixx
        q_dot = torque[1] / Iyy
        r_dot = torque[2] / Izz

        # Costruisci il vettore delle derivate
        state_dot = np.zeros(12)
        state_dot[0:3] = [vx, vy, vz]        # posizione
        state_dot[3:6] = [ax, ay, az]        # velocità
        state_dot[6:9] = [p, q, r]           # angoli
        state_dot[9:12] = [p_dot, q_dot, r_dot]  # velocità angolari

        return state_dot
    
    # === Integration methods ===
    def euler_integration(self, thrust, torque, dt):
        state_dot = self.compute_derivatives(self.clean_state, thrust, torque)
        self.clean_state += dt * state_dot

    def runge_kutta_2_integration(self, thrust, torque, dt):
        state = self.clean_state.copy()
        k1 = self.compute_derivatives(state, thrust, torque)
        k2 = self.compute_derivatives(state + 0.5 * dt * k1, thrust, torque)
        self.clean_state = state + dt * k2

    def runge_kutta_45_integration(self, thrust, torque, dt):
        state = self.clean_state.copy()
        k1 = self.compute_derivatives(state, thrust, torque)
        k2 = self.compute_derivatives(state + dt * k1 * 0.2, thrust, torque)
        k3 = self.compute_derivatives(state + dt * (3*k1 + 9*k2)/40, thrust, torque)
        k4 = self.compute_derivatives(state + dt * (44*k1 - 168*k2 + 160*k3)/45, thrust, torque)
        k5 = self.compute_derivatives(state + dt * (19372*k1 - 76080*k2 + 72960*k3 + 7296*k4)/7290, thrust, torque)
        k6 = self.compute_derivatives(state + dt * (439*k1/216 - 8*k2 + 3680*k3/513 - 845*k4/4104), thrust, torque)
        self.clean_state = state + dt * (25*k1/216 + 1408*k3/2565 + 2197*k4/4104 - k5/5)

        """
         Linearized drone dynamics around hover condition.
        
        Assumptions:
        - Small angles: sin(θ) ≈ θ, cos(θ) ≈ 1
        - Hover condition: thrust ≈ mg
        - Decoupled dynamics for position and attitude
        """

    def linearized_dynamics(self, thrust, torque, dt=None):
        if dt is None:
            dt = self.dt

        if self.integration_method == "euler":
            self.euler_integration(thrust, torque, dt)
        elif self.integration_method == "rk2":
            self.runge_kutta_2_integration(thrust, torque, dt)
        elif self.integration_method == "rk45":
            self.runge_kutta_45_integration(thrust, torque, dt)
        else:
            raise ValueError(f"Unknown integration method: {self.integration_method}")

        ax, ay, az = (thrust / self.mass) * self.clean_state[7], -(thrust / self.mass) * self.clean_state[6], (thrust / self.mass) - self.g
        p, q, r = self.clean_state[9:12]
        true_accel = np.array([ax, ay, az])
        true_gyro = np.array([p, q, r])
        (accel_clean, gyro_clean), (accel_noisy, gyro_noisy) = self.imu_model(true_accel, true_gyro)

        self.clean_state_history.append(self.clean_state.copy())
        self.noisy_state_history.append(self.state.copy())

        
        # PREVIOUS CODE WITH EULER INTEGRATION ONLY
        # # Current state variables
        # x, y, z = self.clean_state[0:3]
        # vx, vy, vz = self.clean_state[3:6]
        # roll, pitch, yaw = self.clean_state[6:9]
        # p, q, r = self.clean_state[9:12]
        
        # # === LINEARIZED POSITION DYNAMICS ===
        # # For small angles, the thrust creates acceleration approximately:
        # # ax ≈ (thrust/mass) * pitch
        # # ay ≈ -(thrust/mass) * roll  
        # # az ≈ (thrust/mass) - g
        
        # ax = (thrust / self.mass) * pitch
        # ay = -(thrust / self.mass) * roll
        # az = (thrust / self.mass) - self.g
        
        # # Update velocities
        # self.clean_state[3] += ax * dt  # vx
        # self.clean_state[4] += ay * dt  # vy
        # self.clean_state[5] += az * dt  # vz
        
        # # Update positions
        # self.clean_state[0] += vx * dt  # x
        # self.clean_state[1] += vy * dt  # y
        # self.clean_state[2] += vz * dt  # z
        
        # # === LINEARIZED ATTITUDE DYNAMICS ===
        # # Simplified angular acceleration (decoupled)
        # # For small angles, the inertia matrix becomes approximately diagonal
        # Ixx, Iyy, Izz = self.inertia[0,0], self.inertia[1,1], self.inertia[2,2]
        
        # p_dot = torque[0] / Ixx
        # q_dot = torque[1] / Iyy  
        # r_dot = torque[2] / Izz
        
        # # Update angular velocities
        # self.clean_state[9] += p_dot * dt   # p
        # self.clean_state[10] += q_dot * dt  # q
        # self.clean_state[11] += r_dot * dt  # r
        
        # # Update angles (small angle assumption: angular velocity ≈ angle rate)
        # self.clean_state[6] += p * dt   # roll
        # self.clean_state[7] += q * dt   # pitch
        # self.clean_state[8] += r * dt   # yaw
        
        # # === SENSOR SIMULATION ===
        # # Get noisy sensor readings
        # true_accel = np.array([ax, ay, az])
        # true_gyro = np.array([p, q, r])
        
        # (accel_clean, gyro_clean), (accel_noisy, gyro_noisy) = self.imu_model(true_accel, true_gyro)
        
        # # Store history
        # self.clean_state_history.append(self.clean_state.copy())
        # self.noisy_state_history.append(self.state.copy())
        

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

    def update_noisy_state_estimate(self, filtered_pos, filtered_vel, filtered_angles, filtered_ang_vel):
        """
        Update the noisy state estimate based on filtered sensor data.
        This represents what an onboard estimator would compute.
        """
        self.state[0:3] = filtered_pos
        self.state[3:6] = filtered_vel
        self.state[6:9] = filtered_angles
        self.state[9:12] = filtered_ang_vel

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
        time = np.arange(clean_array.shape[0]) * self.dt

        plt.figure(figsize=(15, 12))

        # Position comparison
        plt.subplot(4, 1, 1)
        plt.plot(time, clean_array[:, 0], 'b--', label='X Clean', alpha=0.7)
        plt.plot(time, clean_array[:, 1], 'g--', label='Y Clean', alpha=0.7)
        plt.plot(time, clean_array[:, 2], 'r--', label='Z Clean', alpha=0.7)
        plt.plot(time, noisy_array[:, 0], 'b-', label='X Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 1], 'g-', label='Y Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 2], 'r-', label='Z Noisy', alpha=1)
        plt.title('Position (Linearized Model)')
        plt.ylabel('Position (m)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Velocity comparison
        plt.subplot(4, 1, 2)
        plt.plot(time, clean_array[:, 3], 'b--', label='Vx Clean', alpha=0.7)
        plt.plot(time, clean_array[:, 4], 'g--', label='Vy Clean', alpha=0.7)
        plt.plot(time, clean_array[:, 5], 'r--', label='Vz Clean', alpha=0.7)
        plt.plot(time, noisy_array[:, 3], 'b-', label='Vx Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 4], 'g-', label='Vy Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 5], 'r-', label='Vz Noisy', alpha=1)
        plt.title('Velocity (Linearized Model)')
        plt.ylabel('Velocity (m/s)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Orientation comparison
        plt.subplot(4, 1, 3)
        plt.plot(time, clean_array[:, 6], 'b--', label='Roll Clean', alpha=0.7)
        plt.plot(time, clean_array[:, 7], 'g--', label='Pitch Clean', alpha=0.7)
        plt.plot(time, clean_array[:, 8], 'r--', label='Yaw Clean', alpha=0.7)
        plt.plot(time, noisy_array[:, 6], 'b-', label='Roll Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 7], 'g-', label='Pitch Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 8], 'r-', label='Yaw Noisy', alpha=1)
        plt.title('Orientation (Linearized Model)')
        plt.ylabel('Angle (rad)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Angular velocity comparison
        plt.subplot(4, 1, 4)
        plt.plot(time, clean_array[:, 9], 'b--', label='p Clean', alpha=0.7)
        plt.plot(time, clean_array[:, 10], 'g--', label='q Clean', alpha=0.7)
        plt.plot(time, clean_array[:, 11], 'r--', label='r Clean', alpha=0.7)
        plt.plot(time, noisy_array[:, 9], 'b-', label='p Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 10], 'g-', label='q Noisy', alpha=1)
        plt.plot(time, noisy_array[:, 11], 'r-', label='r Noisy', alpha=1)
        plt.title('Angular Velocity (Linearized Model)')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        plt.tight_layout()
        
        plt.show(block=False)
        input("Premi INVIO per chiudere i grafici...")



# Alias for compatibility
DroneModel = LinearizedDroneModel


def run_example_simulation():
    """Example simulation demonstrating the linearized model."""
    # Drone parameters
    drone_mass = 4  # kg
    
    # Create drone model with reduced noise levels for better stability
    drone = LinearizedDroneModel(
        mass=drone_mass,
        inertia=np.diag([0.01, 0.01, 0.02]),  # Typical quadrotor inertia
        accel_variance=np.array([0.05, 0.05, 0.04]),  # Reduced accelerometer noise (m/s²)
        gyro_variance=np.array([0.02, 0.02, 0.015]),  # Reduced gyroscope noise (rad/s)
        dt=0.01
    )

    # Choose integration method from terminal
    integration_method = input("Scegli il metodo di integrazione (euler / rk2 / rk45): ").lower()
    drone.integration_method = integration_method
    
    # Set initial position closer to target to reduce initial error
    drone.clean_state[2] = 0.5  # Start at 0.5m height instead of 0.1m
    drone.state[2] = 0.5
    
    # Simulation parameters
    duration = 50.0  # seconds (shorter for faster testing)
    steps = int(duration / drone.dt)
    
    # Set the reference altitude and attitude
    ref_xyz = np.array([1.0, 0.0, 1.0])  # Desired position (hover at 1m)
    ref_attitude = np.zeros(3)  # Roll, pitch, yaw
    ref_angular_velocity = np.zeros(3)  # p, q, r
    ref_lin_velocity = np.zeros(3)  # vx, vy, vz

    # Create controller instance
    controller = Controllers.Controllers(drone_mass=drone_mass)

    print("DEBUG: File aggiornato")

    print(f"Running linearized simulation for {duration} seconds ({steps} steps)...")
    print(f"Drone mass: {drone_mass} kg")
    print(f"Gravity compensation: {drone_mass * 9.81:.2f} N")
    
    # Initialize filtered states for IMU-based navigation
    filtered_pos = drone.clean_state[0:3].copy()  # Start from actual initial position
    filtered_vel = drone.clean_state[3:6].copy()  # Start from actual initial velocity  
    filtered_angles = drone.clean_state[6:9].copy()  # Start from actual initial angles
    
    # Initialize filtered sensor with initial gravity and zero angular rates
    drone.filtered_sensor[0:3] = np.array([0, 0, drone.g])  # Initial accelerometer (gravity)
    drone.filtered_sensor[3:6] = np.zeros(3)  # Initial gyroscope (no rotation)
    
    # Initialize bias estimates
    estimated_accel_bias = np.zeros(3)
    estimated_gyro_bias = np.zeros(3)
    bias_learning_rate = 0.001  # Increased learning rate for better adaptation
    
    # Initialize complementary filter for attitude estimation
    complementary_alpha = 0.98  # High-pass filter coefficient for gyroscope

    for i in range(steps):
        # First compute control based on current estimates
        forceZ_cmd, torque_cmd = controller.low_level_control(
            pos_desired=ref_xyz,
            vel_desired=ref_lin_velocity,
            att_desired=ref_attitude,
            pos_current=filtered_pos,
            vel_current=filtered_vel,
            att_current=filtered_angles
        )
        
        # Apply control commands
        thrust = forceZ_cmd
        torque = np.array(torque_cmd)

        # Step the simulation (this updates the clean state and generates new sensor data)
        drone.step(thrust, torque)
        
        # Now update estimates based on new sensor readings
        # Extract filtered accelerometer and gyroscope data
        filtered_accel = drone.filtered_sensor[0:3]
        filtered_gyro = drone.filtered_sensor[3:6]

        # Apply bias compensation with better adaptation
        filtered_accel_corrected = filtered_accel - estimated_accel_bias
        filtered_gyro_corrected = filtered_gyro - estimated_gyro_bias

        # === IMPROVED ATTITUDE ESTIMATION ===
        # Use complementary filter for better attitude estimation
        # Integrate gyroscope for short-term accuracy
        gyro_angles = filtered_angles + filtered_gyro_corrected * drone.dt
        
        # Calculate attitude from accelerometer (gravity vector)
        # Only use when total acceleration is close to gravity (hovering)
        accel_magnitude = np.linalg.norm(filtered_accel_corrected)
        if 8.0 < accel_magnitude < 12.0:  # Near 9.81 m/s²
            ax, ay, az = filtered_accel_corrected
            accel_roll = np.arctan2(ay, az)
            accel_pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
            #mag_yaw = np.arctan2(drone.mag_x, drone.mag_y)

            # Complementary filter fusion
            filtered_angles[0] = complementary_alpha * gyro_angles[0] + (1 - complementary_alpha) * accel_roll
            filtered_angles[1] = complementary_alpha * gyro_angles[1] + (1 - complementary_alpha) * accel_pitch
            filtered_angles[2] = gyro_angles[2]  # Pure integration for yaw
        else:
            # When accelerating, rely more on gyroscope
            filtered_angles = gyro_angles
        
        # === IMPROVED VELOCITY AND POSITION ESTIMATION ===
        # Use a conservative approach: limit the use of noisy accelerometer for integration
        # Instead, use model-based prediction with periodic corrections
        
        # Model-based velocity prediction (using last control inputs)
        # This would come from the known thrust and attitude commands
        predicted_accel = np.array([0, 0, (thrust/drone.mass) - drone.g])
        model_vel_update = predicted_accel * drone.dt
        
        # Compensate for gravity and tilt in accelerometer data
        # Transform accelerometer readings to world frame (simplified)
        roll, pitch = filtered_angles[0], filtered_angles[1]
        
        # Gravity compensation in world frame
        accel_world = filtered_accel_corrected.copy()
        #accel_world[2] -= drone.g  # Remove gravity
        
        # Apply tilt compensation (simplified linearized)
        accel_world[0] += drone.g * pitch  # Pitch contributes to forward acceleration
        accel_world[1] -= drone.g * roll   # Roll contributes to lateral acceleration

        # Blend model prediction with sensor measurement (sensor fusion)
        sensor_weight = 0.3  # Lower weight for noisy sensor data
        model_weight = 0.7   # Higher weight for model prediction
        
        accel_vel_correction = (sensor_weight * accel_world + model_weight * predicted_accel) * drone.dt
        filtered_vel += accel_vel_correction

        # strong drift correction to prevent unbounded divergence
        # In practice, this would come from GPS, visual odometry, or other sensors
        clean_vel = drone.get_clean_state()[3:6]
        drift_correction_factor = 0.1  # strong correction
        velocity_error = clean_vel - filtered_vel
        filtered_vel += velocity_error * drift_correction_factor
        
        # Update bias estimates based on persistent velocity errors
        estimated_accel_bias += velocity_error * bias_learning_rate * 0.5

        # Update position with corrected velocity and strong position drift correction
        filtered_pos += filtered_vel * drone.dt
        
        # Very strong position drift correction (simulates external position reference)
        clean_pos = drone.get_clean_state()[0:3]
        position_error = clean_pos - filtered_pos
        position_correction_factor = 0.05  # Much stronger position correction
        filtered_pos += position_error * position_correction_factor
        
        # Angular velocity from filtered gyroscope
        filtered_ang_vel = filtered_gyro_corrected.copy()
        
        # Update gyroscope bias estimates based on angular errors
        clean_angles = drone.get_clean_state()[6:9]
        angle_error = clean_angles - filtered_angles
        angle_correction_factor = 0.05  # Stronger angular correction
        filtered_angles += angle_error * angle_correction_factor
        estimated_gyro_bias += angle_error * bias_learning_rate * 0.2

        # Update the noisy state estimate with filtered values
        drone.update_noisy_state_estimate(filtered_pos, filtered_vel, filtered_angles, filtered_ang_vel)
        
        # Print progress every second
        if i % 100 == 0:
            pos_error = np.linalg.norm(filtered_pos - ref_xyz)
            print(f"Time: {i*drone.dt:.1f}s, Position error: {pos_error:.3f}m")
    
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
