# Extended Kalman Filter (EKF) Implementation

## Overview
This implementation provides a 6-DOF Extended Kalman Filter for drone state estimation using IMU and GPS data. The state vector consists of position and velocity in 3D space.

## State Vector
The state vector `x` is a 6-dimensional vector:
```
x = [px, py, pz, vx, vy, vz]^T
```
where:
- `px, py, pz`: Position in world frame (meters)
- `vx, vy, vz`: Velocity in world frame (m/s)

## System Model

### Prediction Step
The system follows a constant velocity model with acceleration input from IMU:

**State Transition:**
```
x(k+1) = F * x(k) + B * u(k)
```

**State Transition Matrix F:**
```
F = [1  0  0  dt 0  0 ]
    [0  1  0  0  dt 0 ]
    [0  0  1  0  0  dt]
    [0  0  0  1  0  0 ]
    [0  0  0  0  1  0 ]
    [0  0  0  0  0  1 ]
```

**Input Matrix B:**
```
B = [0  0  0]
    [0  0  0]
    [0  0  0]
    [R11*dt R12*dt R13*dt]
    [R21*dt R22*dt R23*dt]
    [R31*dt R32*dt R33*dt]
```
where R is the rotation matrix from body to world frame derived from the quaternion.

**Control Input u:**
```
u = R * a_body
```
where `a_body` is the acceleration measured by the IMU in body frame.

### Update Step
The measurement model assumes direct position measurements from GPS:

**Measurement Model:**
```
z = H * x + v
```

**Measurement Matrix H:**
```
H = [1 0 0 0 0 0]
    [0 1 0 0 0 0]
    [0 0 1 0 0 0]
```

## Jacobians

### State Jacobian (F)
The Jacobian of the state transition function with respect to the state is simply the state transition matrix F, as the system model is linear in the state.

### Input Jacobian (B)
The Jacobian of the state transition function with respect to the input (acceleration) depends on the current orientation:

```cpp
Eigen::MatrixXd compute_input_jacobian(const Eigen::Matrix3d& rotation_matrix, double dt) {
  Eigen::MatrixXd B(6, 3);
  B.setZero();
  
  // Position is not directly affected by acceleration in one time step
  // Velocity is affected by rotated acceleration
  B.block<3, 3>(3, 0) = rotation_matrix * dt;
  
  return B;
}
```

### Measurement Jacobian (H)
For direct position measurements, the Jacobian is constant:
```
H = [1 0 0 0 0 0]
    [0 1 0 0 0 0]
    [0 0 1 0 0 0]
```

## Covariance Matrices

### Process Noise (Q)
Models uncertainty in the system dynamics:
```
Q = σ_process² * I₆ₓ₆
```

### Measurement Noise (R)
Models uncertainty in GPS measurements:
```
R = σ_gps² * I₃ₓ₃
```

## Key Features

1. **Quaternion Integration**: Uses quaternions from IMU for robust orientation representation
2. **Body-to-World Transformation**: Properly transforms IMU accelerations from body frame to world frame
3. **Numerical Stability**: Uses Joseph form for covariance update to maintain positive definiteness
4. **ROS2 Integration**: Seamlessly integrates with ROS2 topics for IMU and GPS data

## Usage

```cpp
// Initialize EKF
auto ekf = std::make_shared<EKF>();

// Set initial state and covariance
Eigen::VectorXd x0(6);
x0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  // [px, py, pz, vx, vy, vz]

Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(6, 6) * 1.0;

Eigen::VectorXd quat0(4);
quat0 << 0.0, 0.0, 0.0, 1.0;  // [x, y, z, w]

ekf->Init(x0, P0, quat0);

// EKF will automatically update with incoming IMU and GPS data
rclcpp::spin(ekf);
```

## Improvements Made

1. **Fixed Syntax Errors**: Corrected matrix indexing and variable declarations
2. **Proper ROS2 Integration**: EKF now inherits from rclcpp::Node
3. **Complete Jacobian Implementation**: Added proper computation of all required Jacobians
4. **Robust State Management**: Added proper time handling and initialization checks
5. **Numerical Stability**: Used Joseph form for covariance updates
6. **Code Organization**: Separated concerns and added helper functions