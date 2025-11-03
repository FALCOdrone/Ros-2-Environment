# Complementary_filter.py
import numpy as np

class AttitudeFilter:
    def __init__(self, dt, complementary_alpha=0.98):
        """
        Initializes the filter.
        
        Args:
            dt (float): The time step (sample rate) in seconds.
            complementary_alpha (float): The filter coefficient (high trusts gyro).
        """
        self.dt = dt
        self.filtered_angles = np.zeros(3)  # [roll, pitch, yaw] -> [phi, theta, psi]
        
        # biases
        #self.estimated_gyro_bias = np.zeros(3) 
        
        self.complementary_alpha = complementary_alpha
        self.one_minus_alpha = 1.0 - complementary_alpha


    def update(self, acc_reading, gyro_reading, mag_reading):
        """
        Updates the attitude estimate using new sensor readings.
        """
        
        # 1. Correct sensor readings for bias (if we were estimating it)
        #gyro_corr = gyro_reading - self.estimated_gyro_bias

        # 2. Gyroscope Integration (Prediction Step)
        # --- CORRECT KINEMATIC INTEGRATION ---
        phi, theta, psi = self.filtered_angles
        p, q, r = gyro_reading

        cos_phi = np.cos(phi)
        sin_phi = np.sin(phi)
        cos_theta = np.cos(theta)
        
        if np.abs(cos_theta) < 0.001:
            # Avoid singularity (Gimbal Lock)
            phi_dot   = p
            theta_dot = q * cos_phi - r * sin_phi
            psi_dot   = 0 # Yaw is indeterminate, trust gyro prediction
        else:
            tan_theta = np.tan(theta)
            phi_dot   = p + q * sin_phi * tan_theta + r * cos_phi * tan_theta
            theta_dot =     q * cos_phi             - r * sin_phi
            psi_dot   =     q * sin_phi / cos_theta + r * cos_phi / cos_theta

        gyro_angles = self.filtered_angles + np.array([phi_dot, theta_dot, psi_dot]) * self.dt

        
        # 3. Accelerometer & Magnetometer Correction (Update Step)
        
        # --- Roll/Pitch from Accelerometer ---
        accel_magnitude = np.linalg.norm(acc_reading)
        
        # Only use accelerometer if magnitude is close to 1g (not in freefall or high-g)
        if 8.0 < accel_magnitude < 12.0:
            ax, ay, az = acc_reading
            
            accel_roll = np.arctan2(ay, az)
            accel_pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
            
            # Complementary Fusion for Roll and Pitch
            self.filtered_angles[0] = (
                self.complementary_alpha * gyro_angles[0]
                + self.one_minus_alpha * accel_roll
            )
            self.filtered_angles[1] = (
                self.complementary_alpha * gyro_angles[1]
                + self.one_minus_alpha * accel_pitch
            )
        else:
            # Strong acceleration, trust only gyroscope integration
            self.filtered_angles[0] = gyro_angles[0]
            self.filtered_angles[1] = gyro_angles[1]

        # --- YAW CORRECTION from Magnetometer ---
        phi_est, theta_est = self.filtered_angles[0], self.filtered_angles[1]
        
        cos_phi = np.cos(phi_est); sin_phi = np.sin(phi_est)
        cos_theta = np.cos(theta_est); sin_theta = np.sin(theta_est)
        
        mx, my, mz = mag_reading
        
        # Tilt-compensated magnetometer
        mag_world_x = mx * cos_theta + my * sin_phi * sin_theta + mz * cos_phi * sin_theta
        mag_world_y = my * cos_phi - mz * sin_phi
        
        mag_yaw = np.arctan2(-mag_world_y, mag_world_x)
        
        # Complementary Fusion for Yaw (handling wrap-around)
        yaw_error = mag_yaw - gyro_angles[2]
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi # Normalize error
            
        self.filtered_angles[2] = gyro_angles[2] + self.one_minus_alpha * yaw_error
        
        # Renormalize the final yaw angle to [-pi, pi]
        self.filtered_angles[2] = (self.filtered_angles[2] + np.pi) % (2 * np.pi) - np.pi

        return self.filtered_angles