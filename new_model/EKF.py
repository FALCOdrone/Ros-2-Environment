# ekf.py
import numpy as np

PI = np.pi

class EKF:
    def __init__(self):
        # internal tuning parameters
        self.dtIMU = 0.01
        self.attitudeTau = 0.5
        self.Nstate = 7

        # process noise stddevs (tune these)
        self.QPosXYStd = 0.1    
        self.QPosZStd  = 0.2    
        self.QVelXYStd = 0.1
        self.QVelZStd  = 0.5
        self.QYawStd   = 0.03

        # GPS measurement noise stddevs (tune these)
        self.GPSPosXStd = 0.005   
        self.GPSPosYStd = 0.005
        self.GPSPosZStd = 0.001   
        self.GPSVelXYStd = 0.005
        self.GPSVelZStd = 0.005


        self.MagYawStd = 0.5

        # attitude estimates
        self.rollEst = 0.0
        self.pitchEst = 0.0
        self.yawEst = 0.0

        # EKF matrices (will be initialized in initialize())
        self.ekfCov = np.eye(self.Nstate)
        self.Q = np.zeros((self.Nstate, self.Nstate))
        self.R_GPS = np.zeros((6,6))
        self.R_Mag = np.zeros((1,1))
        self.R_bar = np.zeros((1,1))

        # state
        self.ekfState = np.zeros(self.Nstate)

        # attitude vector (yaw, pitch, roll)
        self.estAttitude = np.zeros(3)

        # measurement noise R for general use
        self.R = None

    def initialize(self, ini_state, ini_stdDevs):
        """Initialize EKF state and covariances.
           ini_state: array-like length 7
           ini_stdDevs: array-like length 7 (std deviations for initial covariance diag)
        """
        ini_state = np.asarray(ini_state, dtype=float).reshape(self.Nstate)
        ini_stdDevs = np.asarray(ini_stdDevs, dtype=float).reshape(self.Nstate)

        # initial covariance
        self.ekfCov = np.eye(self.Nstate, dtype=float)
        for i in range(self.Nstate):
            self.ekfCov[i,i] = ini_stdDevs[i] * ini_stdDevs[i]

        # Q (process noise for transition model)
        self.Q = np.zeros((self.Nstate, self.Nstate), dtype=float)
        self.Q[0,0] = self.Q[1,1] = (self.QPosXYStd)**2
        self.Q[2,2] = (self.QPosZStd)**2
        self.Q[3,3] = self.Q[4,4] = (self.QVelXYStd)**2
        self.Q[5,5] = (self.QVelZStd)**2
        self.Q[6,6] = (self.QYawStd)**2
        self.Q *= self.dtIMU

        # R_GPS
        self.R_GPS = np.zeros((6,6), dtype=float)
        self.R_GPS[0,0] = (self.GPSPosXStd)**2
        self.R_GPS[1,1] = (self.GPSPosYStd)**2
        self.R_GPS[2,2] = (self.GPSPosZStd)**2
        self.R_GPS[3,3] = self.R_GPS[4,4] = (self.GPSVelXYStd)**2
        self.R_GPS[5,5] = (self.GPSVelZStd)**2

        # Magnetometer measurement covariance
        self.R_Mag = np.zeros((1,1), dtype=float)
        self.R_Mag[0,0] = (self.MagYawStd)**2

        self.R_bar = np.zeros((1,1), dtype=float)
        self.R_bar[0,0] = (0.2)**2

        # Attitude estimation initialization
        self.xt_at = np.zeros(4, dtype=float)
        self.xt_at[0] = 1.0
        self.ekfCov_at = np.eye(4, dtype=float)
        self.Q_at = np.eye(4, dtype=float) * 1e-4
        self.H_at = np.eye(4, dtype=float)
        self.R_at = np.eye(4, dtype=float) * 1e-4

        self.rollEst = 0.0
        self.pitchEst = 0.0
        self.yawEst = 0.0

        # Initial ekf state
        self.ekfState = ini_state.copy()
        self.estAttitude = np.array([self.ekfState[6], self.pitchEst, self.rollEst], dtype=float)

    # - Euler/quaternion conversions
    def Euler3212EP(self, p):
        """Euler ZYX (p = [yaw, pitch, roll]) to quaternion [q0,q1,q2,q3]"""
        c1 = np.cos(p[0]/2); s1 = np.sin(p[0]/2)
        c2 = np.cos(p[1]/2); s2 = np.sin(p[1]/2)
        c3 = np.cos(p[2]/2); s3 = np.sin(p[2]/2)

        q0 = c1*c2*c3 + s1*s2*s3
        q1 = c1*c2*s3 - s1*s2*c3
        q2 = c1*s2*c3 + s1*c2*s3
        q3 = s1*c2*c3 - c1*s2*s3

        q = np.array([q0,q1,q2,q3], dtype=float)   # Normalize: Garantisce quaternione unitario da conversione angolare
        q /= np.linalg.norm(q) 
        return q

    def EPEuler321(self, q):
        """Quaternion [q0,q1,q2,q3] to Euler ZYX -> returns [yaw,pitch,roll]"""
        q0,q1,q2,q3 = q[0], q[1], q[2], q[3]
        yaw = np.arctan2(2*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3)
        val = -2 * (q1*q3 - q0*q2)
        val = np.clip(val, -1.0, 1.0)
        pitch = np.arcsin(val)
        roll = np.arctan2(2*(q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3)
        return np.array([yaw, pitch, roll], dtype=float)


    # BodyRates -> EulerVelocities (uses stored rollEst,pitchEst)
    def BodyRates_to_EulerVelocities(self, pqr):
        """pqr: [p,q,r] body rates (ordered as in C++ function signature)
           returns euler velocities [phi_dot, theta_dot, psi_dot] relative to euler angles
        """
        roll = self.rollEst
        pitch = self.pitchEst

        m = np.zeros((3,3), dtype=float)
        m[0,0] = 1.0
        m[1,0] = 0.0
        m[2,0] = 0.0
        m[0,1] = np.sin(roll) * np.tan(pitch)
        m[0,2] = np.cos(roll) * np.tan(pitch)
        m[1,1] = np.cos(roll)
        m[1,2] = -np.sin(roll)
        m[2,1] = np.sin(roll) / np.cos(pitch)
        m[2,2] = np.cos(roll) / np.cos(pitch)
        return m.dot(pqr)

    def GetRbgPrime(self):
        """Returns the partial derivative of Rbg wrt yaw as in original code.
           Uses estAttitude = [yaw, pitch, roll]
        """
        yaw = self.estAttitude[0]
        pitch = self.estAttitude[1]
        roll = self.estAttitude[2]

        RbgPrime = np.zeros((3,3), dtype=float)
        RbgPrime[0,0] = -np.cos(pitch) * np.sin(yaw)
        RbgPrime[0,1] = -np.sin(roll) * np.sin(pitch) * np.sin(yaw) - np.cos(roll) * np.cos(yaw)
        RbgPrime[0,2] = -np.cos(roll) * np.sin(pitch) * np.sin(yaw) + np.sin(roll) * np.cos(yaw)
        RbgPrime[1,0] = np.cos(pitch) * np.cos(yaw)
        RbgPrime[1,1] = np.sin(roll) * np.sin(pitch) * np.cos(yaw) - np.cos(roll) * np.sin(yaw)
        RbgPrime[1,2] = np.cos(roll) * np.sin(pitch) * np.cos(yaw) + np.sin(roll) * np.sin(yaw)
        return RbgPrime

    def quatRotMat(self, q):
        """Quaternion rotation matrix (from quaternion q = [q0,q1,q2,q3])"""
        q = np.asarray(q, dtype=float)
        q /= np.linalg.norm(q)        # Normalize: Assicura matrice di rotazione ortogonale

        q0,q1,q2,q3 = q
        M = np.zeros((3,3), dtype=float)
        M[0,0] = 1 - 2*q2*q2 - 2*q3*q3
        M[0,1] = 2*q1*q2 - 2*q0*q3
        M[0,2] = 2*q1*q3 + 2*q0*q2
        M[1,0] = 2*q1*q2 + 2*q0*q3
        M[1,1] = 1 - 2*q1*q1 - 2*q3*q3
        M[1,2] = 2*q2*q3 - 2*q0*q1
        M[2,0] = 2*q1*q3 - 2*q0*q2
        M[2,1] = 2*q2*q3 + 2*q0*q1
        M[2,2] = 1 - 2*q1*q1 - 2*q2*q2
        return M

    # --- EKF predict/update ---
    def predict(self, acc, gyro, dt):
        """Prediction step:
           acc: body-frame accel vector (3,)
           gyro: body-frame gyro (unused here except for attitude in other routines)
           dt: delta time
        """
        acc = np.asarray(acc, dtype=float).reshape(3)
        gyro = np.asarray(gyro, dtype=float).reshape(3)

        predictedState = self.ekfState.copy()

        # rotation from body to inertial using quaternion xt_at
        R_bg = self.quatRotMat(self.xt_at)
        inertial_accel = R_bg.dot(acc)
        # inertial_accel[2] -= -9.81   remove gravity
    
        # kinematic update
        predictedState[0] = self.ekfState[0] + self.ekfState[3] * dt    # x(k+1) = x(k) + vx*dt
        predictedState[1] = self.ekfState[1] + self.ekfState[4] * dt    # y(k+1) = y(k) + vy*dt
        predictedState[2] = self.ekfState[2] + self.ekfState[5] * dt    # z(k+1) = z(k) + vz*dt
        predictedState[3] = self.ekfState[3] + inertial_accel[0] * dt   # vx(k+1) = vx(k) + ax*dt
        predictedState[4] = self.ekfState[4] + inertial_accel[1] * dt   # vy(k+1) = vy(k) + ay*dt
        predictedState[5] = self.ekfState[5] + inertial_accel[2] * dt   # vz(k+1) = vz(k) + az*dt
        # yaw (state[6]) not predicted here (attitude handled separately)

        # Jacobian gPrime (7x7 identity with partials)
        gPrime = np.eye(self.Nstate, dtype=float)
        gPrime[0,3] = dt
        gPrime[1,4] = dt
        gPrime[2,5] = dt

        RbgPrime = self.GetRbgPrime()
        helper_matrix = RbgPrime.dot(acc)
        gPrime[3,6] = helper_matrix[0] * dt
        gPrime[4,6] = helper_matrix[1] * dt
        gPrime[5,6] = helper_matrix[2] * dt

        gTranspose = gPrime.T
        self.ekfCov = gPrime.dot(self.ekfCov).dot(gTranspose) + self.Q
        self.ekfState = predictedState

    def update_ekf(self, z, H, R, zFromX, dt):
        """Generic EKF update step"""
        z = np.asarray(z, dtype=float).reshape(-1)
        zFromX = np.asarray(zFromX, dtype=float).reshape(-1)
        H = np.asarray(H, dtype=float)
        R = np.asarray(R, dtype=float)

        assert z.shape[0] == H.shape[0]
        assert self.Nstate == H.shape[1]
        assert z.shape[0] == R.shape[0] and z.shape[0] == R.shape[1]
        assert z.shape[0] == zFromX.shape[0]

        toInvert = H.dot(self.ekfCov).dot(H.T) + R
        K = self.ekfCov.dot(H.T).dot(np.linalg.inv(toInvert))

        self.ekfState = self.ekfState + K.dot((z - zFromX))

        eye = np.eye(self.Nstate, dtype=float)
        self.ekfCov = (eye - K.dot(H)).dot(self.ekfCov)

    def updateFromMag(self, magYaw, dt):
        """Update yaw from magnetometer (magYaw)"""
        z = np.array([magYaw], dtype=float)
        zFromX = np.array([self.ekfState[6]], dtype=float)

        hPrime = np.zeros((1, self.Nstate), dtype=float)
        hPrime[0,6] = 1.0

        # wrap angle from mag to be nearest to state yaw (handle discontinuity)
        diff = z[0] - zFromX[0]
        # wrap to [-pi, pi]
        diff = (diff + PI) % (2*PI) - PI
        z[0] = zFromX[0] + diff

        self.update_ekf(z, hPrime, self.R_Mag, zFromX, dt)
        self.estAttitude[0] = self.ekfState[6]
        self.xt_at = self.Euler3212EP(self.estAttitude)

    def updateFromGps(self, pos, vel, dt):  # to be called in the Drone model to take the filtered x,y,z,vx,vy,vz
        """pos, vel: arrays length 3"""
        pos = np.asarray(pos, dtype=float).reshape(3)
        vel = np.asarray(vel, dtype=float).reshape(3)

        z = np.zeros(6, dtype=float)
        z[0:3] = pos
        z[3:6] = vel

        hPrime = np.zeros((6, self.Nstate), dtype=float)
        for i in range(6):
            if i < self.Nstate:
                hPrime[i,i] = 1.0

        zFromX = np.zeros(6, dtype=float)
        for i in range(6):
            zFromX[i] = self.ekfState[i]

        self.update_ekf(z, hPrime, self.R_GPS, zFromX, dt)

    def updateFromBar(self, altitude, dt):
        z = np.array([altitude], dtype=float)
        zFromX = np.array([self.ekfState[2]], dtype=float)

        hprime = np.zeros((1, self.Nstate), dtype=float)
        hprime[0,2] = 1.0

        self.update_ekf(z, hprime, self.R_bar, zFromX, dt)

    # FOR NOW WE ARE MAINTAINING YAW = 0
    # def yawFromMag(self, mag, quat):
    #     """Compute yaw from magnetometer and quaternion.
    #        mag: object-like with x,y attributes or array-like [mx,my,mz]
    #        quat: object-like with w,x,y,z or array-like [w,x,y,z]
    #     """
    #     # accept either arrays or simple objects
    #     if hasattr(mag, '__len__'):
    #         Bx, By = mag[0], mag[1]
    #     else:
    #         Bx, By = mag.x, mag.y

    #     if hasattr(quat, '__len__'):
    #         quat_readings = np.asarray(quat, dtype=float).reshape(4)
    #     else:
    #         quat_readings = np.array([quat.w, quat.x, quat.y, quat.z], dtype=float)

    #     euler_angles = self.EPEuler321(quat_readings)
    #     pitch = euler_angles[1]
    #     roll = euler_angles[2]

    #     yawMag = np.arctan2(By * np.cos(roll) - Bx * np.sin(roll),
    #                         Bx * np.cos(pitch) + By * np.sin(pitch) * np.sin(roll))
    #     return yawMag

   
    ### DA RIVEDERE SE VA BENE ###

    def complementary_filter_attitude(self, acc, gyro, dt):  # to be called in the Drone model to take the filtered roll, pitch, yaw
        """
        Compute roll, pitch and yaw filtered by accelerometers and gyroscope
        using the complementary filter
        """
        # Initialization
        if not hasattr(self, "filtered_angles"):
            self.filtered_angles = np.zeros(3)
            self.estimated_accel_bias = np.zeros(3)
            self.estimated_gyro_bias = np.zeros(3)
            self.bias_learning_rate = 0.001
            self.complementary_alpha = 0.96
        
        # Correzione bias
        acc_corr = acc - self.estimated_accel_bias
        gyro_corr = gyro - self.estimated_gyro_bias
        
        # Integrazione giroscopio
        gyro_angles = self.filtered_angles + gyro_corr * dt
        
        # Calcolo roll/pitch da accelerometro
        accel_magnitude = np.linalg.norm(acc_corr)
        if 8.0 < accel_magnitude < 12.0:  # close to g = 9.81
            ax, ay, az = acc_corr
            accel_roll = np.arctan2(ay, az)
            accel_pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
            
            # Fusion with complementary filter
            self.filtered_angles[0] = (
                self.complementary_alpha * gyro_angles[0]
                + (1 - self.complementary_alpha) * accel_roll
            )
            self.filtered_angles[1] = (
                self.complementary_alpha * gyro_angles[1]
                + (1 - self.complementary_alpha) * accel_pitch
            )
            self.filtered_angles[2] = gyro_angles[2]  # pure yaw from gyroscope
        else:
            # Strong acceleration, so trust only gyroscope
            self.filtered_angles = gyro_angles
        
        # Update bias
        self.estimated_accel_bias += (acc_corr - acc) * self.bias_learning_rate * 0.5
        self.estimated_gyro_bias += (gyro_corr - gyro) * self.bias_learning_rate * 0.2

        return self.filtered_angles