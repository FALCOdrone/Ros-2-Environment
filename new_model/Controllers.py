import numpy as np
# import scipy.linalg  # Will install later if needed

"""
DRONE MODEL OVERVIEW:
====================

1. COORDINATE SYSTEM:
   - World frame: X (North), Y (East), Z (Down) - NED convention
   - Body frame: X (forward), Y (right), Z (down)

2. STATE VECTOR (12 states):
   Position:    [x, y, z]        (m) - world frame
   Velocity:    [vx, vy, vz]     (m/s) - world frame  
   Attitude:    [roll, pitch, yaw] (rad) - Euler angles
   Ang. rates:  [p, q, r]        (rad/s) - body frame angular velocities

3. CONTROL INPUTS:
   Thrust:      F_z (N) - total thrust (always upward in body frame)
   Torques:     [τx, τy, τz] (N⋅m) - roll, pitch, yaw torques

4. DYNAMICS MODEL:
   
   A) LINEARIZED POSITION DYNAMICS (small angle approximation):
      ẍ ≈ (F_z/m) * θ        (pitch creates forward acceleration)
      ÿ ≈ -(F_z/m) * φ       (roll creates lateral acceleration)  
   /home/lorenzo/polimi/mobile_robots   z̈ ≈ (F_z/m) - g       (thrust minus gravity)
   
   B) LINEARIZED ATTITUDE DYNAMICS:
      φ̇ = p, θ̇/home/lorenzo/polimi/mobile_robots = q, ψ̇ = r   (small angle rates)
      ṗ = τx/Ixx, q̇ = τy/Iyy, ṙ = τz/Izz  (decoupled inertia)

5. CONTROL STRATEGY:
   - Position control (X,Y): Uses attitude (roll/pitch) as intermediate PID control
   - Altitude control (Z): Direct thrust control with gravity compensation
   - Attitude control: PID or LQR controller for optimal stability and performance

6. SENSOR MODEL:
   - IMU: Provides noisy accelerometer and gyroscope measurements
   - Accelerometer: a_measured = a_true + noise + biase
   - Gyroscope: ω_measured = ω_true + noise + bias
"""

   
class Controllers:
    def __init__(self, drone_mass=1.0):
        # PID gains for position control - better tuned for linearized model
        self.kp_pos = np.array([3.0, 2.0, 3.0])  # Proportional gains for x, y, z (reduced Z)
        self.ki_pos = np.array([1.1, 1.1, 0.05])  # Integral gains for x, y, z (reduced Z integral)
        self.kd_pos = np.array([5.0, 1.0, 1.5])  # Derivative gains for x, y, z (reduced Z)

        # PID gains for attitude control - better tuned
        self.kp_att = np.array([5.0, 5.0, 1.5])  # Proportional gains for roll, pitch, yaw
        self.ki_att = np.array([1.1, 1.1, 0.05])  # Integral gains for roll, pitch, yaw
        self.kd_att = np.array([1.5, 1.0, 0.5])  # Derivative gains for roll, pitch, yaw

        # State variables for integral terms
        self.pos_error_integral = np.zeros(3)
        self.att_error_integral = np.zeros(3)
        self.prev_pos_error = np.zeros(3)
        self.prev_att_error = np.zeros(3)
        self.dt = 0.01  # Time step for integration
        
        # Drone parameters
        self.drone_mass = drone_mass
        self.gravity = 9.81
        
        # Saturation limits
        self.max_thrust = 42.183  # Maximum thrust (N) - reduced for safety (for a single motor = 42.183N)
        #self.max_thrust = 25.0
        self.min_thrust = 0.0   # Minimum thrust (N)
        self.max_torque = 0.5   # Maximum torque (N⋅m) - reduced for stability

    def low_level_control(self, pos_desired, vel_desired, att_desired, pos_current, vel_current, att_current):
        # ===== POSITION CONTROL =====
        # Z control with gravity compensation
        posZ_error = pos_desired[2] - pos_current[2]
        velZ_error = vel_desired[2] - vel_current[2]

        self.pos_error_integral[2] += posZ_error * self.dt
        
        # Anti-windup: limit integral term - tighter limits
        self.pos_error_integral[2] = np.clip(self.pos_error_integral[2], -1.0, 1.0)
        #self.pos_error_integral[2] = np.clip(self.pos_error_integral[2], -2.0, 2.0)

        # Add gravity compensation (mass * gravity) to the thrust command
        gravity_compensation = self.drone_mass * self.gravity
        forceZ_cmd = (self.kp_pos[2] * posZ_error + 
                     self.ki_pos[2] * self.pos_error_integral[2] + 
                     self.kd_pos[2] * velZ_error + gravity_compensation)
        
        # Saturate thrust command
        forceZ_cmd = np.clip(forceZ_cmd, self.min_thrust, self.max_thrust)
        
        # ===== X-Y POSITION CONTROL (through attitude) =====
        # For linearized model: ax ≈ (thrust/mass) * pitch, ay ≈ -(thrust/mass) * roll
        # So: desired_pitch = (desired_ax * mass) / thrust, desired_roll = -(desired_ay * mass) / thrust
        
        # X and Y position errors
        posX_error = pos_desired[0] - pos_current[0]
        posY_error = pos_desired[1] - pos_current[1]
        velX_error = vel_desired[0] - vel_current[0]  
        velY_error = vel_desired[1] - vel_current[1]
        
        # Update integral terms for X,Y
        self.pos_error_integral[0] += posX_error * self.dt
        self.pos_error_integral[1] += posY_error * self.dt
        
        # Anti-windup for X,Y - tighter limits
      #   self.pos_error_integral[0] = np.clip(self.pos_error_integral[0], -1.0, 1.0)
      #   self.pos_error_integral[1] = np.clip(self.pos_error_integral[1], -1.0, 1.0)
        self.pos_error_integral[0] = np.clip(self.pos_error_integral[0], -0.5, 0.5)
        self.pos_error_integral[1] = np.clip(self.pos_error_integral[1], -0.5, 0.5)

        # Desired accelerations in X and Y - with limits
        desired_ax = (self.kp_pos[0] * posX_error + 
                     self.ki_pos[0] * self.pos_error_integral[0] + 
                     self.kd_pos[0] * velX_error)
        
        desired_ay = (self.kp_pos[1] * posY_error + 
                     self.ki_pos[1] * self.pos_error_integral[1] + 
                     self.kd_pos[1] * velY_error)
        
        # Limit desired accelerations to reasonable values
        max_accel = 0.5  # m/s²
        #max_accel = 2.0  # m/s² - increased for better responsiveness
        desired_ax = np.clip(desired_ax, -max_accel, max_accel)
        desired_ay = np.clip(desired_ay, -max_accel, max_accel)
        
        # Convert desired accelerations to desired angles
        # Prevent division by zero and ensure minimum thrust
        safe_thrust = max(forceZ_cmd, 5.0)  # Minimum 5N thrust
        desired_pitch = (desired_ax * self.drone_mass) / safe_thrust
        desired_roll = -(desired_ay * self.drone_mass) / safe_thrust  # Note the negative sign
        
        # Limit desired angles to conservative values for stability
        max_angle = 0.1  # ±5.73 degrees approximately -> 0.1 rad
        #max_angle = 0.2 # ±11.46 degrees approximately -> 0.2 rad
        desired_pitch = np.clip(desired_pitch, -max_angle, max_angle)
        desired_roll = np.clip(desired_roll, -max_angle, max_angle)
        
        # Update desired attitude to include position control
        att_desired_modified = np.array([desired_roll, desired_pitch, att_desired[2]])
        
        # ===== ATTITUDE CONTROL (PID) =====
        att_error = att_desired_modified - att_current
        
        # Wrap angle errors to [-π, π]
        att_error = np.arctan2(np.sin(att_error), np.cos(att_error))
        
        att_derivative = (att_error - self.prev_att_error) / self.dt
        self.prev_att_error = att_error
        
        self.att_error_integral += att_error * self.dt
        
        # Anti-windup: limit integral terms
        #self.att_error_integral = np.clip(self.att_error_integral, -1.0, 1.0)
        self.att_error_integral = np.clip(self.att_error_integral, -0.5, 0.5)
        
        torque_cmd = (self.kp_att * att_error + 
                      self.ki_att * self.att_error_integral + 
                      self.kd_att * att_derivative)
        
        # Saturate torque commands
        torque_cmd = np.clip(torque_cmd, -self.max_torque, self.max_torque)
        
        return forceZ_cmd, torque_cmd
    
    def reset_integral_terms(self):
        """Reset all integral terms (useful for simulation resets)."""
        self.pos_error_integral = np.zeros(3)
        self.att_error_integral = np.zeros(3)
        self.prev_pos_error = np.zeros(3)
        self.prev_att_error = np.zeros(3)
    
    def set_gains(self, kp_pos=None, ki_pos=None, kd_pos=None, 
                  kp_att=None, ki_att=None, kd_att=None):
        """Update controller gains dynamically."""
        if kp_pos is not None:
            self.kp_pos = np.array(kp_pos)
        if ki_pos is not None:
            self.ki_pos = np.array(ki_pos)
        if kd_pos is not None:
            self.kd_pos = np.array(kd_pos)
        if kp_att is not None:
            self.kp_att = np.array(kp_att)
        if ki_att is not None:
            self.ki_att = np.array(ki_att)
        if kd_att is not None:
            self.kd_att = np.array(kd_att)
    
    def get_control_info(self):
        """Return current controller state for debugging."""
        return {
            'pos_error_integral': self.pos_error_integral.copy(),
            'att_error_integral': self.att_error_integral.copy(),
            'gains': {
                'kp_pos': self.kp_pos,
                'ki_pos': self.ki_pos,
                'kd_pos': self.kd_pos,
                'kp_att': self.kp_att,
                'ki_att': self.ki_att,
                'kd_att': self.kd_att,
            }
        }