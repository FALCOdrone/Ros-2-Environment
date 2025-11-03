# ekf.py
import numpy as np

class PositionVelocityEKF:
    def __init__(self, drone_model):
        self.drone_model = drone_model
        self.Nstate = 6    # State: [x, y, z, vx, vy, vz]

        # EKF matrices
        self.ekfCov = np.eye(self.Nstate)
        self.Q = np.zeros((self.Nstate, self.Nstate))
        self.R_GPS = np.zeros((6, 6))
        self.R_bar = np.zeros((1, 1))

        # state
        self.ekfState = np.zeros(self.Nstate)
        
        # Store gravity for convenience
        self.g_world = np.array([0, 0, -self.drone_model.g]) # Z is up, so gravity is negative

    def initialize(self, ini_state, ini_stdDevs, gps_pos_var, gps_vel_var, baro_var):
        """Initialize EKF state and covariances.
           ini_state: array-like length 6 [x, y, z, vx, vy, vz]
           ini_stdDevs: array-like length 6 (std deviations for initial covariance diag)
           gps_pos_var: array-like length 3 (GPS position variance [x,y,z])
           gps_vel_var: array-like length 3 (GPS velocity variance [vx,vy,vz])
           baro_var: float (Barometer variance)
        """
        ini_state = np.asarray(ini_state, dtype=float).reshape(self.Nstate)
        ini_stdDevs = np.asarray(ini_stdDevs, dtype=float).reshape(self.Nstate)

        # initial covariance
        self.ekfCov = np.diag(ini_stdDevs**2)

        # Q process noise covariance (tune these!)
        pos_proc_noise = 0.2   # increased from 0.05
        vel_proc_noise = 0.2   # increased from 0.2
        self.Q = np.diag([
            pos_proc_noise**2, pos_proc_noise**2, pos_proc_noise**2,
            vel_proc_noise**2, vel_proc_noise**2, vel_proc_noise**2
        ])

        # GPS measurement noise stddevs
        gps_var = np.concatenate([gps_pos_var, gps_vel_var])
        self.R_GPS = np.diag(gps_var)

        # Barometer measurement covariance
        self.R_bar = np.array([[baro_var]])

        # Initial ekf state
        self.ekfState = ini_state.copy()

    # --- EKF predict/update ---
    def predict(self, acc_body, full_attitude, dt):
        """
        Prediction step:
        acc_body: body-frame accel vector [ax, ay, az] (from IMU)
        full_attitude: [roll, pitch, yaw] (from Attitude Filter)
        dt: delta time
        """

        # Debug helper: check accelerometer magnitude (should be close to gravity when hovering)
        try:
            import logging
            mag = np.linalg.norm(acc_body)
            logging.debug(f"EKF: accel_body magnitude = {mag:.3f}, dt={dt:.4f}")
        except Exception:
            pass
        
        # --- 1. Rotate Accelerometer to World Frame ---
        # The IMU measures (True Acceleration - Gravity)
        # We need to rotate this measurement and add gravity back
        # to get the True World-Frame Acceleration.
        
        # Get Body-to-World rotation matrix
        R_w_b = self.get_body_to_world_matrix(full_attitude)
        
        # Rotate measurement and add gravity
        # a_world = R_w_b * (acc_body) + g_world
        acc_true_world = R_w_b.dot(acc_body) + self.g_world
    
        # --- 2. State Prediction (Constant Acceleration Model) ---
        predictedState = self.ekfState.copy()
        
        # x = x + v*dt + 0.5*a*dt^2
        predictedState[0:3] = (self.ekfState[0:3] + 
                               self.ekfState[3:6] * dt + 
                               0.5 * acc_true_world * dt**2)
        
        # v = v + a*dt
        predictedState[3:6] = self.ekfState[3:6] + acc_true_world * dt

        # --- 3. Covariance Prediction ---
        # Jacobian of the state transition function f(x) w.r.t. state x
        gPrime = np.eye(self.Nstate, dtype=float)
        gPrime[0,3] = dt
        gPrime[1,4] = dt
        gPrime[2,5] = dt

        self.ekfCov = gPrime.dot(self.ekfCov).dot(gPrime.T) + self.Q * dt
        self.ekfState = predictedState

    def update_ekf(self, z, H, R, zFromX, dt):
        """Generic EKF update step"""
        z = np.asarray(z, dtype=float).reshape(-1)
        zFromX = np.asarray(zFromX, dtype=float).reshape(-1)

        # Innovation (error)
        y = z - zFromX
        
        toInvert = H.dot(self.ekfCov).dot(H.T) + R
        K = self.ekfCov.dot(H.T).dot(np.linalg.inv(toInvert))

        self.ekfState = self.ekfState + K.dot(y)

        eye = np.eye(self.Nstate, dtype=float)
        self.ekfCov = (eye - K.dot(H)).dot(self.ekfCov)

    def updateFromGps(self, pos, vel, dt):
        """Correct EKF state using GPS position and velocity"""
        pos = np.asarray(pos, dtype=float).reshape(3)
        vel = np.asarray(vel, dtype=float).reshape(3)

        z = np.concatenate([pos, vel]) # Measurement vector [x,y,z,vx,vy,vz]

        # Measurement Jacobian (H)
        hPrime = np.eye(6, self.Nstate, dtype=float)

        # Predicted measurement
        zFromX = self.ekfState[0:6]

        self.update_ekf(z, hPrime, self.R_GPS, zFromX, dt)

    def updateFromBar(self, altitude, dt):
        """Correct EKF state using Barometer altitude"""
        z = np.array([altitude], dtype=float)
        
        # Measurement Jacobian (H)
        # z = H * x. z is altitude, which is state[2] (z-position)
        hprime = np.zeros((1, self.Nstate), dtype=float)
        hprime[0,2] = 1.0

        # Predicted measurement
        zFromX = np.array([self.ekfState[2]], dtype=float)

        self.update_ekf(z, hprime, self.R_bar, zFromX, dt)

    # --- Helper Functions ---
    def get_body_to_world_matrix(self, euler_angles):
        """
        Creates a Body-to-World (R_w_b) rotation matrix
        from Euler angles [roll, pitch, yaw]
        """
        roll, pitch, yaw = euler_angles
        
        cr, cp, cy = np.cos(roll), np.cos(pitch), np.cos(yaw)
        sr, sp, sy = np.sin(roll), np.sin(pitch), np.sin(yaw)

        # ZYX convention for R_b_w (World-to-Body)
        R_b_w = np.array([
            [cy*cp, sy*cp, -sp],
            [cy*sp*sr - sy*cr, sy*sp*sr + cy*cr, cp*sr],
            [cy*sp*cr + sy*sr, sy*sp*cr - cy*sr, cp*cr]
        ])
        
        # R_w_b is the transpose
        return R_b_w.T
    
    def getHprime_from_gps(self):
        """Returns the measurement Jacobian H for GPS updates."""
        hPrime = np.eye(6, self.Nstate, dtype=float)
        return hPrime