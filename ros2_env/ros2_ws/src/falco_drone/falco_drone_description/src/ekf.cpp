#include "falco_drone_description/ekf.h"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/PointStamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

/*
  Note: This ekf version estimates position and velocity,
  assuming that the imu measurements are already been filtered correctly. This simplifies the EKF implementation,
  as it does not need to handle non-linearities in the model. This simplification is valid, since
  the IMU data is already filtered internally by the MPU6050.h library, which is used to read the IMU data.
*/

EKF::EKF() : Node("ekf_node"), first_prediction_(true) {
  // Initialize subscribers
  std::string imu_topic = "/real_time/imu/out";
  std::string gps_topic = "/real_time/gps_position/out";
  std::string baro_topic = "/real_time/barometer/out";
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  
  // Subscribe to IMU, GPS and Baro
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, qos,
    std::bind(&EKF::imu_callback, this, std::placeholders::_1));

  gps_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    gps_topic, qos,
    std::bind(&EKF::gps_callback, this, std::placeholders::_1));

  baro_subscriber_ = this->create_subscription<sensor_msgs::msg::Barometer>(
    baro_topic, qos,
    std::bind(&EKF::baro_callback, this, std::placeholders::_1));
  
  // Initialize publishers
  odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/real_time/ekf/odometry", 10);
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/real_time/ekf/pose", 10);
  twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/real_time/ekf/twist", 10);

  RCLCPP_INFO(this->get_logger(), "EKF Node initialized");
  
  // Initialize process and measurement noise covariances -> TODO: tune those values
  Q_ = Eigen::MatrixXd::Identity(7, 7) * 0.01;  // Process noise
  R_gps = Eigen::MatrixXd::Identity(6, 6) * 0.1;   // Measurement noise (GPS position)
  R_baro = Eigen::MatrixXd::Identity(1, 1) * 0.5;  // Measurement noise (Barometer altitude)
}

EKF::~EKF() {}

void EKF::Init(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0, const Eigen::VectorXd &quat_0) {
  x_ = x0;  // State vector: [px, py, pz, vx, vy, vz]
  P_ = P0;  // Covariance matrix
  
  // Initialize quaternion from vector (assuming [x, y, z, w] format)
  if (quat_0.size() == 4) {
    quat_ = Eigen::Quaterniond(quat_0(3), quat_0(0), quat_0(1), quat_0(2));
  } else {
    quat_ = Eigen::Quaterniond::Identity();
  }
  quat_.normalize();
  
  last_time_ = this->get_clock()->now();
  first_prediction_ = false;
}

void EKF::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Update quaternion from IMU
  quat_ = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, 
                            msg->orientation.y, msg->orientation.z);
  quat_.normalize();
  
  // Store IMU acceleration in body frame
  imu_accel_ = Eigen::Vector3d(msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z);
  
  imu_gyro_ = Eigen::Vector3d(msg->angular_velocity.x,
                             msg->angular_velocity.y,
                             msg->angular_velocity.z);
  
  // Trigger prediction step
  if (!first_prediction_) {
    Predict();
  }
}

void EKF::gps_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  // Convert GPS data to local coordinates and update EKF
  // For simplicity, assuming GPS provides position measurements
  // In a real implementation, you would convert lat/lon to local coordinates
  
  Eigen::VectorXd z(3);
  z << msg->point.x, msg->point.y, msg->point.z;  // Placeholder - needs coordinate conversion

  Update(z, H_GPS(z), R_gps);
}

void EKF::baro_callback(const sensor_msgs::msg::Barometer::SharedPtr msg) {
  // Barometer data can be used to update altitude
  // Assuming barometer provides altitude in meters
  Eigen::VectorXd z(1);
  z << msg->altitude;  // Placeholder - needs coordinate conversion
  
  // Update EKF with barometer measurement
  Update(z, H_Baro(z), R_baro);
}

Eigen::MatrixXd EKF::H_Baro(const Eigen::VectorXd &z) {
  // Measurement model: H maps state to measurement
  // Assuming we measure altitude directly (Barometer)
  Eigen::MatrixXd H(1, 6);
  H.setZero();
  H(0, 2) = 1;  // Measure altitude (pz) only
  return H;
}

Eigen::MatrixXd EKF::compute_state_jacobian(double dt) {
  // State transition matrix F for constant velocity model
  // State: [px, py, pz, vx, vy, vz, yaw_rate]
  Eigen::MatrixXd F(7, 7);
  F << 1, 0, 0, dt, 0,  0, 0,
       0, 1, 0, 0,  dt, 0, 0,
       0, 0, 1, 0,  0,  dt, 0,
       0, 0, 0, 1,  0,  0, 0,
       0, 0, 0, 0,  1,  0, 0,
       0, 0, 0, 0,  0,  1, 0,
       0, 0, 0, 0,  0,  0, 1;
  return F;
}

Eigen::MatrixXd EKF::compute_input_jacobian(const Eigen::Matrix3d& rotation_matrix, double dt) {
  // Input matrix B for acceleration input
  // Maps body frame acceleration to world frame velocity change
  Eigen::MatrixXd B(7, 3);
  B.setZero();
  
  // Position is not directly affected by acceleration in one time step
  // Velocity is affected by rotated acceleration
  B.block<3, 3>(3, 0) = rotation_matrix * dt;
  
  return B;
}

void EKF::Predict() {
  if (first_prediction_) {
    return;
  }
  
  rclcpp::Time curr_time = this->get_clock()->now();
  double dt = (curr_time - last_time_).seconds();
  
  if (dt <= 0 || dt > 1.0) {  // Sanity check
    last_time_ = curr_time;
    return;
  }
  
  last_time_ = curr_time;
  
  // Get rotation matrix from quaternion
  Eigen::Matrix3d rotation_matrix = quat_.toRotationMatrix();
  
  // Convert body frame acceleration to world frame
  Eigen::Vector3d world_accel = rotation_matrix * imu_accel_;
  
  // Compute Jacobians
  Eigen::MatrixXd F = compute_state_jacobian(dt);
  Eigen::MatrixXd B = compute_input_jacobian(rotation_matrix, dt);
  
  // Predict state
  // x_k+1 = F * x_k + B * u_k
  Eigen::VectorXd x_pred = F * x_ + B * world_accel;
  
  // Predict covariance
  // P_k+1 = F * P_k * F^T + Q
  Eigen::MatrixXd P_pred = F * P_ * F.transpose() + Q_;
  
  // Update state and covariance
  x_ = x_pred;
  P_ = P_pred;
  
  // Publish updated state estimates
  publish_pose();
  publish_twist();
}

Eigen::MatrixXd EKF::H_GPS(const Eigen::VectorXd &z) {
  // Measurement model: H maps state to measurement
  // Assuming we measure position directly (GPS)
  Eigen::MatrixXd H(3, 6);
  H.setZero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // Measure position only
  return H;
}

Eigen::MatrixXd EKF::H_baro(const Eigen::VectorXd &z) {
  // Measurement model: H maps state to measurement
  // Assuming we measure altitude directly (Barometer)
  Eigen::MatrixXd H(1, 6);
  H.setZero();
  H(0, 2) = 1;  // Measure altitude (pz) only
  return H;
}

void EKF::Update(const Eigen::VectorXd &z, Eigen::MatrixXd H, Eigen::MatrixXd R) {

  // Predicted measurement
  Eigen::VectorXd z_pred = H * x_;
  
  // Innovation (measurement residual)
  Eigen::VectorXd y = z - z_pred;
  
  // Innovation covariance
  Eigen::MatrixXd S = H * P_ * H.transpose() + R;

  // Kalman gain
  Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
  
  // Update state estimate
  x_ = x_ + K * y;
  
  // Update covariance estimate (Joseph form for numerical stability)
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();
  
  // Publish updated state estimates after measurement update
  publish_pose();
  publish_twist();
}

// TODO: Implement the update of position and velocity through the visual odometry algorithm
// to better reduce the uncertainty in the state estimation

void EKF::publish_pose() {
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  
  // Header
  pose_msg.header.stamp = this->get_clock()->now();
  pose_msg.header.frame_id = "odom";
  
  // Position
  pose_msg.pose.pose.position.x = x_(0);
  pose_msg.pose.pose.position.y = x_(1);
  pose_msg.pose.pose.position.z = x_(2);
  
  // Orientation
  pose_msg.pose.pose.orientation.w = quat_.w();
  pose_msg.pose.pose.orientation.x = quat_.x();
  pose_msg.pose.pose.orientation.y = quat_.y();
  pose_msg.pose.pose.orientation.z = quat_.z();
  
  // Position covariance (3x3 -> 9 elements from top-left of P_)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      pose_msg.pose.covariance[i * 6 + j] = P_(i, j);
    }
  }
  
  pose_publisher_->publish(pose_msg);
}

void EKF::publish_twist() {
  geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
  
  // Header
  twist_msg.header.stamp = this->get_clock()->now();
  twist_msg.header.frame_id = "base_link";
  
  // Linear velocity
  twist_msg.twist.twist.linear.x = x_(3);
  twist_msg.twist.twist.linear.y = x_(4);
  twist_msg.twist.twist.linear.z = x_(5);
  
  // Angular velocity (from IMU)
  twist_msg.twist.twist.angular.x = imu_gyro_(0);
  twist_msg.twist.twist.angular.y = imu_gyro_(1);
  twist_msg.twist.twist.angular.z = imu_gyro_(2);
  
  // Velocity covariance (3x3 -> 9 elements from bottom-right of P_)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      twist_msg.twist.covariance[i * 6 + j] = P_(i + 3, j + 3);
    }
  }
  
  twist_publisher_->publish(twist_msg);
}