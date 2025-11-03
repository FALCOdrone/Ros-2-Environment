import numpy as np

class SensorSuite:
    """
    Simulates a sensor suite (IMU, Mag, GPS) with different update rates,
    noise, and bias models.
    
    This class reads the "ground truth" from the provided drone_model
    and generates realistic sensor readings.
    """
    def __init__(self, drone_model, sim_dt, 
                 mag_variance=np.array([1e-5, 1e-5, 1e-5]),
                 gps_pos_variance=np.array([0.5, 0.5, 1.5]),
                 gps_vel_variance=np.array([0.1, 0.1, 0.1])):
        
        self.drone_model = drone_model
        self.sim_dt = sim_dt
        self.g = drone_model.g  # Get gravity constant from drone model

        # --- Sensor Rates ---
        self.imu_mag_period = 1.0 / 100.0  # 100 Hz
        self.gps_period = 1.0 / 1.0       # 1 Hz
        
        # --- Timers ---
        # We use timers to track when to generate a new reading
        self._imu_mag_timer = 0.0
        self._gps_timer = 0.0
        
        # --- Noise Parameters ---
        self.accel_variance = self.drone_model.accel_variance
        self.gyro_variance = self.drone_model.gyro_variance
        self.mag_variance = mag_variance
        self.gps_pos_variance = gps_pos_variance
        self.gps_vel_variance = gps_vel_variance

        # --- Biases (simulating a random walk, neglected for simplicity) ---
        # self._accel_bias = np.random.normal(0, 0.02, 3)
        # self._gyro_bias = np.random.normal(0, 0.01, 3)
        # self._mag_bias = np.random.normal(0, 0.005, 3)
        
        # --- "Hardware" Data Buffers ---
        # These store the last *generated* measurement
        self.latest_accel = np.zeros(3)
        self.latest_gyro = np.zeros(3)
        self.latest_mag = np.zeros(3)
        self.latest_gps_pos = np.zeros(3)
        self.latest_gps_vel = np.zeros(3)

        # Store previous imu readings and alpha coefficient for low pass filter
        self.prev_accel = np.zeros(3)
        self.prev_gyro = np.zeros(3)
        self.alpha = 0.4  # Low pass filter coefficient

        # --- Data Ready Flags ---
        # These are set to True when a new measurement is generated
        # The estimator is responsible for setting them back to False
        self.new_imu_available = False
        self.new_mag_available = False
        self.new_gps_available = False

        # --- Magnetometer Field (simulated) ---
        # A simple approximation: points North (X) and slightly down (Z)
        self._mag_field_world = np.array([0.8, 0.0, 0.2])
        self._mag_field_world /= np.linalg.norm(self._mag_field_world)

    def _update_biases(self):
        """Simulate a slow random walk for sensor biases."""
        self._accel_bias += np.random.normal(0, 0.0001, 3) * self.sim_dt
        self._gyro_bias += np.random.normal(0, 0.00005, 3) * self.sim_dt
        self._mag_bias += np.random.normal(0, 0.00002, 3) * self.sim_dt

    def _get_rotation_matrix(self, roll, pitch, yaw):
        """Calculates the World-to-Body rotation matrix (R_b_w)."""
        cr, cp, cy = np.cos(roll), np.cos(pitch), np.cos(yaw)
        sr, sp, sy = np.sin(roll), np.sin(pitch), np.sin(yaw)
        
        # This is R_b_w (transforms a vector from World to Body frame)
        R_b_w = np.array([
            [cy*cp, sy*cp, -sp],
            [cy*sp*sr - sy*cr, sy*sp*sr + cy*cr, cp*sr],
            [cy*sp*cr + sy*sr, sy*sp*cr - cy*sr, cp*cr]
        ])
        return R_b_w

    def _generate_imu_mag_data(self):
        """Generates new IMU and Mag data based on ground truth."""
        # 1. Get True State from the simulation
        clean_state = self.drone_model.get_clean_state()
        
        # true_accel is WORLD frame linear acceleration (ax, ay, az)
        a_world = self.drone_model.true_accel
        
        # true_gyro is BODY frame angular velocity (p, q, r)
        true_gyro_body = self.drone_model.true_gyro
        
        roll, pitch, yaw = clean_state[6:9]
        
        # 2. Calculate World-to-Body rotation
        R_b_w = self._get_rotation_matrix(roll, pitch, yaw)
        
        # 3. Simulate Accelerometer
        # Transform world-frame linear acceleration to body-frame
        a_body = R_b_w @ a_world
        
        # Gravity vector in world frame (Z is up, so g is negative)
        g_world = np.array([0, 0, -self.g])
        
        # Transform gravity vector to body-frame
        g_body = R_b_w @ g_world
        
        # Accelerometer measures non-gravitational acc (a_body)
        # minus the gravity vector (g_body).
        accel_reading_true = a_body - g_body
        
        # 4. Simulate Magnetometer
        # Rotate the world magnetic field into the body frame
        mag_reading_true = R_b_w @ self._mag_field_world
        
        # 5. Add Noise and Bias to all readings
        accel_noise = np.random.normal(0, np.sqrt(self.accel_variance), 3)
        gyro_noise = np.random.normal(0, np.sqrt(self.gyro_variance), 3)
        mag_noise = np.random.normal(0, np.sqrt(self.mag_variance), 3)
        
        self.latest_accel = accel_reading_true + accel_noise
        self.latest_gyro = true_gyro_body + gyro_noise
        self.latest_mag = mag_reading_true + mag_noise

        self.new_imu_available = True
        self.new_mag_available = True

    def _generate_gps_data(self):
        """Generates new GPS data based on ground truth."""
        # 1. Get True State
        clean_state = self.drone_model.get_clean_state()
        true_pos = clean_state[0:3]
        true_vel = clean_state[3:6]
        
        # 2. Add Noise
        pos_noise = np.random.normal(0, np.sqrt(self.gps_pos_variance), 3)
        vel_noise = np.random.normal(0, np.sqrt(self.gps_vel_variance), 3)
        
        self.latest_gps_pos = true_pos + pos_noise
        self.latest_gps_vel = true_vel + vel_noise
        
        self.new_gps_available = True

    def update(self, current_time):
        """
        This method should be called at every simulation step (e.g., 100 Hz).
        It updates internal timers and generates new sensor data
        if their respective periods have elapsed.
        """
        
        # Always update biases at the high rate
        # self._update_biases()

        # Check if it's time for an IMU/Mag update
        # Using >= allows it to catch up if a sim step is missed
        if current_time >= self._imu_mag_timer:
            self._imu_mag_timer = current_time + self.imu_mag_period
            self._generate_imu_mag_data()
            
        # Check if it's time for a GPS update
        if current_time >= self._gps_timer:
            self._gps_timer = current_time + self.gps_period
            self._generate_gps_data()
                
    # --- Public Data Access Methods ---
    
    def get_imu_reading(self):
        """Returns (accel, gyro). Resets the data available flag."""
        self.new_imu_available = False

        # Low pass filter the imu readings
        self.latest_accel = self.alpha * self.latest_accel + (1 - self.alpha) * self.prev_accel
        self.latest_gyro = self.alpha * self.latest_gyro + (1 - self.alpha) * self.prev_gyro

        self.prev_accel = self.latest_accel.copy()
        self.prev_gyro = self.latest_gyro.copy()

        return self.latest_accel.copy(), self.latest_gyro.copy()
        
    def get_mag_reading(self):
        """Returns (mag). Resets the data available flag."""
        self.new_mag_available = False
        return self.latest_mag.copy()

    def get_gps_reading(self):
        """Returns (pos, vel). Resets the data available flag."""
        self.new_gps_available = False
        return self.latest_gps_pos.copy(), self.latest_gps_vel.copy()