import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D  # registers 3D projection

# OSQP + sparse
import osqp
import scipy.sparse as sparse

# --- importing drone model ---
import model

# ------------------ Simulation parameters ------------------
dt = 0.05
N = 30              # MPC horizon

# Model object and linearization/dynamics
model = model.DroneModel(Ts=dt)
A, B = model.dynamics()      # A: (n,n), B: (n,m)
n = A.shape[0]
m = B.shape[1]

# Weights (terminal-goal)
q_pos_stage = 0.1       # before: 0.5
q_vel_stage = 0.05      # before: 0.05
Q_stage = np.diag([q_pos_stage, q_pos_stage, q_pos_stage,
                   q_vel_stage, q_vel_stage, q_vel_stage,
                   0.0, 0.0])[:n,:n]   
Qf = np.diag([1000.0, 1000.0, 1000.0, 50.0, 50.0, 50.0, 0.0, 0.0])[:n,:n]
R = 0.5 * np.eye(m)

a_max = 4.0 # max acceleration in m/s^2
lb = np.kron(np.ones(N), -a_max * np.ones(m)) # lower bound on u (3N,)
ub = np.kron(np.ones(N), a_max * np.ones(m))  # upper bound on u (3N,)

# Simulation time
T_sim = 4.0        # seconds
sim_steps = int(np.ceil(T_sim / dt))

# Recall of the trajectory states: [x,y,z,vx,vy,vz,yaw,yaw_dot] (example n=8)
# Initial and goal states
x0 = np.array([0.0]*n)
x0[:6] = np.array([0.0,0.0,0.0, 0.0,0.0,0.0])   # if relevant
x_goal = np.array([1.5, 1.0, 0.8] + [0.0]*(n-3))
if x_goal.shape[0] != n:
    # pad or truncate as needed
    tmp = np.zeros(n)
    tmp[:min(len(x_goal), n)] = x_goal[:min(len(x_goal), n)]
    x_goal = tmp

# Precompute prediction matrices (A,B constant, lti system)
Sx0, Su = model.build_prediction_matrices(A, B, N)  # assume model provides this

# Create an initial H,g using x0 (first iteration)
H_init, g_init, Qbar = model.build_qp_in_u_terminal_goal(Q_stage, R, Qf, Sx0, Su, x0, x_goal)

# Ensure H is symmetric and numerically PSD: take half (H+H.T)/2 and small regularizer
P_init = (H_init + H_init.T) * 0.5 # make sure symmetric for OSQP
eps = 1e-8
P_init += eps * np.eye(P_init.shape[0])

# Convert to sparse CSC the hessian and constraint matrix for OSQP
P_sparse = sparse.csc_matrix(P_init) # sparced for OSQP in order to save memory while optimizing
A_osqp = sparse.eye(P_init.shape[0], format='csc')  # identity for bounds l <= U <= u
# for now we add only bound constraints on u

# OSQP setup
solver = osqp.OSQP()
solver.setup(P=P_sparse, q=g_init, A=A_osqp, l=lb, u=ub,
             warm_start=True, verbose=False, polish=True)

# ------------------ Storage for results ------------------
x_actual = np.zeros((sim_steps+1, n))
x_actual[0] = x0.copy()
planned_paths = []
planned_us = []

# Warm start variable for u (start with zeros)
u_prev = np.zeros(m * N)

# ------------------ Receding-horizon loop with OSQP ------------------
for t in range(sim_steps):
    # build/update H and g for current x_actual[t]
    H, g, Qbar = model.build_qp_in_u_terminal_goal(Q_stage, R, Qf, Sx0, Su, x_actual[t], x_goal)

    # Make sure P is symmetric and add tiny regularization if needed
    P = (H + H.T) * 0.5
    P += eps * np.eye(P.shape[0])
    # convert to sparse once here (only necessary if structure changes)
    # If H structure is constant across iterations we could create P_sparse once and skip this
    P_sparse = sparse.csc_matrix(P)

    # If P structure is constant you can avoid updating P each time; update q is sufficient.
    # Here we do a cheap check: if structure changed, update P; otherwise only q
    try:
        solver.update(q=g)          # update linear term
    except Exception:
        # If OSQP complains (e.g. P changed significantly), update P and q
        solver.update(Px=P_sparse.data, q=g)

    # warm start with previous solution (very helpful)
    solver.warm_start(x=u_prev)

    # solve
    res = solver.solve()
    if res.info.status != 'solved':
        # fallback: try to use the raw vector (avoid crashing)
        print(f"OSQP didn't solve at step {t}, status: {res.info.status}. Using warm-started u_prev.")
        u_opt = u_prev.copy()
    else:
        u_opt = res.x.copy()

    # reshape controls and reconstruct planned positions
    u_seq = u_opt.reshape(N, m)
    X_pred = Sx0 @ x_actual[t] + Su @ u_opt
    positions = np.zeros((N, 3))
    for k in range(N):
        xk = X_pred[k*n:(k+1)*n]
        positions[k, :] = xk[:3]
    planned_paths.append(positions.copy())
    planned_us.append(u_seq.copy())

    # apply first control and simulate one step
    u0 = u_seq[0]
    x_next = A @ x_actual[t] + B @ u0
    x_actual[t+1] = x_next

    # warm start: shift previous solution and append last
    u_prev = np.vstack((u_seq[1:], u_seq[-1:])).reshape(-1)


# -------------------------- Plot optimal input sequences, actual trajectory and planned paths ------------------
time_u = np.arange(N) * dt
u_final = planned_us[-1]  # last planned inputs
fig_u, axs_u = plt.subplots(m, 1, figsize=(8, 6))

print("\nClose the figures for running the animation\n")

# enlarge figure for better visibility
fig_u.set_size_inches(10, 6)

# compute y-limits based on data and a_max
u_abs_max = np.max(np.abs(u_final))
y_margin = 0.25 * u_abs_max
ymin, ymax = -u_abs_max - y_margin, u_abs_max + y_margin

for i in range(m):
    axs_u[i].plot(time_u, u_final[:, i], label=f'Input u[{i}]', linewidth=2)
    axs_u[i].axhline(a_max, color='r', linestyle='--', label='Max limit' if i==0 else "")
    axs_u[i].axhline(-a_max, color='r', linestyle='--')
    axs_u[i].set_ylabel(f'u[{i}] [m/s²]')
    axs_u[i].set_xlim(time_u[0], time_u[-1])
    axs_u[i].set_ylim(ymin, ymax)
    axs_u[i].grid()
    if i == 0:
        axs_u[i].legend()
axs_u[-1].set_xlabel('Time [s]')
plt.suptitle('Final planned optimal input sequence')
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()

# Plot actual trajectory
time_x = np.arange(sim_steps+1) * dt
fig_x, axs_x = plt.subplots(3, 1, figsize=(8, 8))
state_labels = ['X [m]', 'Y [m]', 'Z [m]']
for i in range(3):
    axs_x[i].plot(time_x, x_actual[:, i], label=f'State x[{i}]')
    axs_x[i].axhline(x_goal[i], color='g', linestyle='--', label='Goal' if i==0 else "")
    axs_x[i].set_ylabel(state_labels[i])
    axs_x[i].grid()
    if i == 0:
        axs_x[i].legend()
axs_x[-1].set_xlabel('Time [s]')
plt.suptitle('Actual trajectory over time')
plt.show()

# ------------------ Build animation data (same plotting as before) ------------------
frames = sim_steps
fig = plt.figure(figsize=(9, 7))
ax = fig.add_subplot(111, projection='3d')
margin = 0.4
ax.set_xlim(min(x0[0], x_goal[0]) - margin, max(x0[0], x_goal[0]) + margin)
ax.set_ylim(min(x0[1], x_goal[1]) - margin, max(x0[1], x_goal[1]) + margin)
ax.set_zlim(min(x0[2], x_goal[2]) - margin, max(x0[2], x_goal[2]) + margin + 0.2)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title('Closed-loop receding-horizon MPC (OSQP): actual path vs planned futures')

ax.scatter([x_goal[0]], [x_goal[1]], [x_goal[2]], marker='X', s=100, label='goal', color='C3')

actual_line, = ax.plot([], [], [], '-', linewidth=2, label='actual path', color='C0')
drone_point, = ax.plot([], [], [], 'o', markersize=6, color='C0')
planned_line, = ax.plot([], [], [], '--', linewidth=1.5, label='current plan', color='C1')
ax.legend()

def init_anim():
    actual_line.set_data([], [])
    actual_line.set_3d_properties([])
    drone_point.set_data([], [])
    drone_point.set_3d_properties([])
    planned_line.set_data([], [])
    planned_line.set_3d_properties([])
    return actual_line, drone_point, planned_line

def animate(i):
    idx = i
    xs = x_actual[:idx+1, 0]
    ys = x_actual[:idx+1, 1]
    zs = x_actual[:idx+1, 2]
    actual_line.set_data(xs, ys)
    actual_line.set_3d_properties(zs)
    drone_point.set_data([x_actual[idx,0]], [x_actual[idx,1]])
    drone_point.set_3d_properties([x_actual[idx,2]])
    plan = planned_paths[idx] if idx < len(planned_paths) else planned_paths[-1]
    planned_line.set_data(plan[:,0], plan[:,1])
    planned_line.set_3d_properties(plan[:,2])
    return actual_line, drone_point, planned_line

anim = animation.FuncAnimation(fig, animate, init_func=init_anim,
                               frames=frames, interval=dt*1000, blit=True)

print("Sim steps:", sim_steps)
print("Final actual position:", np.round(x_actual[-1,:3],4))
print("Goal position:", x_goal[:3])
print("Distance to goal at end:", np.linalg.norm(x_actual[-1,:3]-x_goal[:3]))

plt.show()
