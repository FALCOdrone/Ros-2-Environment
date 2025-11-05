import numpy as np
#import matplotlib
#matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import Controllers
import Complementary_filter
import EKF, Sensors

# [LinearizedDroneModel class remains the same]
# ... (all the code for LinearizedDroneModel) ...
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

        # GPS noise variances
        self.gps_pos_variance = np.array([0.001, 0.001, 0.001])
        self.gps_vel_variance = np.array([0.001, 0.001, 0.001])

        # Magnetometer noise variances
        self.mag_variance = np.array([0.01, 0.01, 0.01])

        self.true_accel = np.zeros(3)
        self.true_gyro = np.zeros(3)

        # Gravity constant
        self.g = 9.81
    
    def compute_derivatives(self, state, thrust, torque):
        """
        Compute the time derivatives, using M*g for horizontal acceleration 
        to maintain stability near hover.
        """
        # Initial state variable assignments
        x, y, z = state[0:3]
        vx, vy, vz = state[3:6]
        roll, pitch, yaw = state[6:9]
        p, q, r = state[9:12]

        # --- Stability Fix: Enforce Linearization Assumption ---
        # The horizontal dynamics (ax, ay) are linearized around T = M*g. 
        # Use M*g for horizontal forces, but use 'thrust' for vertical force.
        HOVER_THRUST = self.mass * self.g 
        
        MAX_ANGLE_ACCEL = 0.5 
        roll_limited = np.clip(roll, -MAX_ANGLE_ACCEL, MAX_ANGLE_ACCEL)
        pitch_limited = np.clip(pitch, -MAX_ANGLE_ACCEL, MAX_ANGLE_ACCEL)

        # Horizontal Acceleration (Use HOVER_THRUST for stability)
        ax = (HOVER_THRUST / self.mass) * pitch_limited
        ay = -(HOVER_THRUST / self.mass) * roll_limited
        
        # Vertical Acceleration (Use actual commanded thrust)
        az = (thrust / self.mass) - self.g 

        # ... (rest of the code for p_dot, q_dot, r_dot remains the same, 
        #      including the clipping of angular accelerations if you added it)
        
        MAX_ANG_ACCEL = 20.0 # Limit in rad/s^2 
        Ixx, Iyy, Izz = self.inertia[0,0], self.inertia[1,1], self.inertia[2,2]
        
        p_dot = np.clip(torque[0] / Ixx, -MAX_ANG_ACCEL, MAX_ANG_ACCEL)
        q_dot = np.clip(torque[1] / Iyy, -MAX_ANG_ACCEL, MAX_ANG_ACCEL)
        r_dot = np.clip(torque[2] / Izz, -MAX_ANG_ACCEL, MAX_ANG_ACCEL)
        
        # Build the derivative vector
        state_dot = np.zeros(12)
        state_dot[0:3] = [vx, vy, vz]
        state_dot[3:6] = [ax, ay, az]  # <-- Now using stabilized ax, ay
        state_dot[6:9] = [p, q, r]
        state_dot[9:12] = [p_dot, q_dot, r_dot] 

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

        roll, pitch, yaw = self.clean_state[6:9]
        p, q, r = self.clean_state[9:12]

        # --- Make sensor accel consistent with compute_derivatives ---
        # Use the same hover-based linearization as compute_derivatives
        HOVER_THRUST = self.mass * self.g
        MAX_ANGLE_ACCEL = 0.5
        roll_limited = np.clip(roll, -MAX_ANGLE_ACCEL, MAX_ANGLE_ACCEL)
        pitch_limited = np.clip(pitch, -MAX_ANGLE_ACCEL, MAX_ANGLE_ACCEL)

        # Horizontal acceleration uses hover thrust (consistent with compute_derivatives)
        ax = (HOVER_THRUST / self.mass) * pitch_limited
        ay = -(HOVER_THRUST / self.mass) * roll_limited
        # Vertical acceleration uses actual thrust
        az = (thrust / self.mass) - self.g
        
        self.true_accel = np.array([ax, ay, az])
        self.true_gyro = np.array([p, q, r])

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
        """
        Get current sensor readings as they would appear from real IMU. 
        NOTE: In this simplified model, the "state" holds the *estimate*,
              and the sensor readings are generated in Sensors.py.
              This function is mostly a placeholder/for plotting.
        """
        # Return the 'true' instantaneous values from the dynamics
        return {
            'gyro': self.true_gyro.copy(),
            'accel': self.true_accel.copy() # Note: This should include a noise model in a real sim
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
        plt.plot(time, noisy_array[:, 0], 'b-', label='X Filtered', alpha=1)
        plt.plot(time, noisy_array[:, 1], 'g-', label='Y Filtered', alpha=1)
        plt.plot(time, noisy_array[:, 2], 'r-', label='Z Filtered', alpha=1)
        plt.title('Position (Linearized Model)')
        plt.ylabel('Position (m)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Velocity comparison
        plt.subplot(4, 1, 2)
        plt.plot(time, clean_array[:, 3], 'b--', label='Vx Clean', alpha=0.7)
        plt.plot(time, clean_array[:, 4], 'g--', label='Vy Clean', alpha=0.7)
        plt.plot(time, clean_array[:, 5], 'r--', label='Vz Clean', alpha=0.7)
        plt.plot(time, noisy_array[:, 3], 'b-', label='Vx Filtered', alpha=1)
        plt.plot(time, noisy_array[:, 4], 'g-', label='Vy Filtered', alpha=1)
        plt.plot(time, noisy_array[:, 5], 'r-', label='Vz Filtered', alpha=1)
        plt.title('Velocity (Linearized Model)')
        plt.ylabel('Velocity (m/s)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Orientation comparison
        plt.subplot(4, 1, 3)
        plt.plot(time, clean_array[:, 6], 'b--', label='Roll Clean', alpha=0.7)
        plt.plot(time, clean_array[:, 7], 'g--', label='Pitch Clean', alpha=0.7)
        plt.plot(time, clean_array[:, 8], 'r--', label='Yaw Clean', alpha=0.7)
        plt.plot(time, noisy_array[:, 6], 'b-', label='Roll Filtered', alpha=1)
        plt.plot(time, noisy_array[:, 7], 'g-', label='Pitch Filtered', alpha=1)
        plt.plot(time, noisy_array[:, 8], 'r-', label='Yaw Filtered', alpha=1)
        plt.title('Orientation (Linearized Model)')
        plt.ylabel('Angle (rad)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # Angular velocity comparison
        plt.subplot(4, 1, 4)
        plt.plot(time, clean_array[:, 9], 'b--', label='p Clean', alpha=0.7)
        plt.plot(time, clean_array[:, 10], 'g--', label='q Clean', alpha=0.7)
        plt.plot(time, clean_array[:, 11], 'r--', label='r Clean', alpha=0.7)
        plt.plot(time, noisy_array[:, 9], 'b-', label='p Filtered', alpha=1)
        plt.plot(time, noisy_array[:, 10], 'g-', label='q Filtered', alpha=1)
        plt.plot(time, noisy_array[:, 11], 'r-', label='r Filtered', alpha=1)
        plt.title('Angular Velocity (Linearized Model)')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.legend()
        plt.grid(True, alpha=0.3)

        plt.tight_layout()
        
        plt.show()
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
        dt=0.001
    )

    # Initialize sensor suite
    # Note: IMU runs at 100 Hz (0.01s period)
    sensor_meas = Sensors.SensorSuite(
        drone,
        sim_dt=drone.dt,
        mag_variance=drone.mag_variance,
        gps_pos_variance=drone.gps_pos_variance,
        gps_vel_variance=drone.gps_vel_variance
    )

    # 1. Initialize Attitude Filter
    complementary_filter = Complementary_filter.AttitudeFilter(dt=sensor_meas.imu_mag_period, complementary_alpha=0.98)

    # 2. Initialize EKF
    ekf = EKF.PositionVelocityEKF(drone)
    ekf.initialize(
        ini_state=[0, 0, 0, 0, 0, 0],
        ini_stdDevs=[0.1, 0.1, 0.2, 0.1, 0.1, 0.1],
        gps_pos_var=drone.gps_pos_variance,
        gps_vel_var=drone.gps_vel_variance,
        baro_var=0.05
    )

    # Choose integration method from terminal
    integration_method = input("Scegli il metodo di integrazione (euler / rk2 / rk45): ").lower()
    drone.integration_method = integration_method
    
    # Set initial position closer to target
    drone.clean_state[2] = 0.0  
    drone.state[2] = 0.0

    # Simulation parameters
    duration = 50.0 
    steps = int(duration / drone.dt)
    
    # Set the reference altitude and attitude
    ref_xyz = np.array([0.0, 0.0, 0.0])  # Desired position (hover at 0m)
    ref_attitude = np.zeros(3)  # Roll, pitch, yaw
    ref_lin_velocity = np.zeros(3)  # vx, vy, vz
    
    print("DEBUG: File aggiornato")
    print(f"Running linearized simulation for {duration} seconds ({steps} steps)...")
    
    # Initialize filtered state variables (Attitude is now managed by CF)
    filtered_angles = drone.clean_state[6:9].copy() 
    filtered_ang_vel = np.zeros(3)
    
    # Track the last time a prediction/update occurred for the EKF to calculate dt
    last_ekf_time = 0.0
    controller = Controllers.Controllers(drone_mass=drone_mass)
    controller.dt = sensor_meas.imu_mag_period  # Match controller dt to IMU update rate

    for i in range(steps):

        current_time = i * drone.dt

        # Sensor readings
        sensor_meas.update(current_time)
        
        # --- ATTITUDE ESTIMATION (100 Hz) ---
        if sensor_meas.new_imu_available:
            
            # 1. Get IMU and Mag data
            accel_meas, gyro_meas = sensor_meas.get_imu_reading()
            mag_meas = sensor_meas.get_mag_reading()
            
            # 2. Update Attitude Filter
            # The CF uses its internal dt (0.01s) for integration
            filtered_angles = complementary_filter.update(accel_meas, gyro_meas, mag_meas)
            filtered_ang_vel = gyro_meas.copy()
            
            # 3. EKF Prediction using Attitude
            # dt_ekf is the time since the last IMU/EKF step (should be 0.01s)
            dt_ekf = current_time - last_ekf_time
            if dt_ekf > 0: # Ensure we only predict after a real time step
                # The EKF needs accel_body and the full_attitude [r, p, y]
                ekf.predict(accel_meas, filtered_angles, dt_ekf)
                last_ekf_time = current_time

            # Clear IMU flag only after prediction
            sensor_meas.new_imu_available = False

        # --- EKF CORRECTION (GPS/Barometer) ---
        # The sensor suite tracks the GPS period 1Hz
        if sensor_meas.new_gps_available:
            gps_pos, gps_vel = sensor_meas.get_gps_reading()

            print("EKF P diag before GPS:", np.sqrt(np.diag(ekf.ekfCov)))
            z = np.concatenate([gps_pos, gps_vel])
            hprime_gps = ekf.getHprime_from_gps() if hasattr(ekf,'getHprime_from_gps') else None
            if hprime_gps is not None:
                residual = z - hprime_gps @ ekf.ekfState
                print("GPS residual:", residual, "norm:", np.linalg.norm(residual))
            ekf.updateFromGps(gps_pos, gps_vel, drone.dt)
            print("EKF P diag after GPS:", np.sqrt(np.diag(ekf.ekfCov)))

            # The GPS update doesn't need a dt argument in the EKF update step, 
            # but we use the sim dt for convenience if needed for other updates
            ekf.updateFromGps(gps_pos, gps_vel, drone.dt) 
            sensor_meas.new_gps_available = False

        # --- CONTROL ---
        # compute control based on current estimates
        forceZ_cmd, torque_cmd = controller.low_level_control(
            pos_desired=ref_xyz,
            vel_desired=ref_lin_velocity,
            att_desired=ref_attitude,
            # Use EKF estimates for position/velocity
            pos_current=ekf.ekfState[0:3],
            vel_current=ekf.ekfState[3:6],
            # Use CF estimate for attitude
            att_current=filtered_angles
        )
        
        # Apply control commands
        #thrust = forceZ_cmd
        #torque = np.array(torque_cmd)

        # TEMP TEST: disable controller to isolate estimator
        thrust = drone.mass * drone.g
        torque = np.zeros(3)

        # Step the simulation
        drone.step(thrust, torque)

        # Update the noisy state estimate with filtered values for plotting
        drone.update_noisy_state_estimate(ekf.ekfState[0:3], ekf.ekfState[3:6], filtered_angles, filtered_ang_vel)

        # Print progress every second
        if i % 1000 == 0:
            pos_error = np.linalg.norm(ekf.ekfState[0:3] - ref_xyz)
            print(f"Time: {current_time:.1f}s, EKF Position error: {pos_error:.3f}m")
    
    print("Simulation complete!")
    
    # Calculate final errors
    final_clean_error = np.linalg.norm(drone.get_clean_state()[:3] - ref_xyz)
    final_noisy_error = np.linalg.norm(drone.get_noisy_state()[:3] - ref_xyz)
    print(f"Final clean position error: {final_clean_error:.3f} m")
    print(f"Final filtered position error: {final_noisy_error:.3f} m")
    
    # Plot results
    drone.plot_state()
    
    return drone


if __name__ == "__main__":
    # Run example simulation
    drone = run_example_simulation()