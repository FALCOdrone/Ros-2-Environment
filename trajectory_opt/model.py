import numpy as np

class DroneModel:
    def __init__(self,
                 mass=4.0, # in kg
                 inertia=np.diag([0.05, 0.05, 0.1]), # in kg*m^2
                 max_thrust=40.0, # in Newtons
                 Ts=0.01):
        self.mass = mass
        self.inertia = inertia
        self.max_thrust = max_thrust
        self.Ts = Ts

    def dynamics(self):
        # Placeholder for drone dynamics computation

        """
        GENERAL STATES: [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
        MPC TRAJECTORY STATES: [x, y, z, vx, vy, vz, yaw, yaw_dot]
        TRAJECTORY CONTROL INPUTS: [a_x, a_y, a_z, yaw_ddot]
        """
        A = np.eye(8)  # Placeholder for state matrix
        B = np.zeros((8, 5))  # Placeholder for input matrix

        A[0, 3] = self.Ts
        A[1, 4] = self.Ts
        A[2, 5] = self.Ts

        B[3, 0] = B[4, 1] = B[5, 2] = B[6, 3] = B[7, 4] = self.Ts

        return A, B

    def build_prediction_matrices(self, A, B, N):
        """ Build prediction matrices for MPC over horizon N.
            
        Args:
            A: State transition matrix
            B: Control input matrix
            N: Prediction horizon
        Returns:
            Sx0: State prediction matrix
            Su: Input prediction matrix
        """

        n = A.shape[0]
        m = B.shape[1]

        Sx0 = np.zeros((n * N, n))
        Su = np.zeros((n * N, m * N))

        A_pows = [np.eye(n)]

        for k in range(1, N+1):
            A_pows.append(A_pows[-1] @ A)

        for k in range(1, N+1):
            Sx0[(k-1)*n:k*n, :] = A_pows[k]
            for j in range(k):
                Su[(k-1)*n:k*n, j*m:(j+1)*m] = A_pows[k-j-1] @ B
        return Sx0, Su 

    def build_qp_in_u_terminal_goal(self, Q, R, Qf, Sx0, Su, x0, x_goal):
        """ Build QP matrices for MPC with terminal goal.

        Args:
            Q: State cost matrix
            R: Input cost matrix
            Qf: Terminal state cost matrix
            Sx0: State prediction matrix
            Su: Input prediction matrix
            x0: Initial state
            x_goal: Goal state
        Returns:
            H: Quadratic cost matrix
            g: Linear cost vector
            Qbar: Block diagonal state cost matrix

        
        Description:
            The cost function is defined as:
                J = 0.5 * U.T @ H @ U + g.T @ U
            where U is the stacked control input vector over the prediction horizon.
            Therefore we are minimizing the deviation from the goal state at the final time step.
            """

        n = Q.shape[0]
        N = Sx0.shape[0] // n # prediction horizon

        Qbar = np.zeros((n*N, n*N))

        for k in range(N-1):
            Qbar[k*n:(k+1)*n, k*n:(k+1)*n] = Q
        Qbar[(N-1)*n:(N)*n, (N-1)*n:(N)*n] = Qf

        m = R.shape[0]
        Rbar = np.zeros((m*N, m*N))

        for k in range(N):
            Rbar[k*m:(k+1)*m, k*m:(k+1)*m] = R

        H = Su.T @ Qbar @ Su + Rbar
        Xref = np.zeros(n*N)
        Xref[(N-1)*n:(N)*n] = x_goal

        g = Su.T @ Qbar @ (Sx0 @ x0 - Xref)
        return H, g, Qbar       
    