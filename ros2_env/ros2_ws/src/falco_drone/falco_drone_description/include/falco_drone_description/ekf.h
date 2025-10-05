// Copyright 2023 Georg Novotny
//
// Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/gpl-3.0.en.html
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EKF_H
#define EKF_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/barometer.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>

class EKF : public rclcpp::Node {
public:
    EKF();
    virtual ~EKF();
    
    void Init(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0, const Eigen::VectorXd &quat_0);
    
protected:
    // Callback functions
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void gps_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void baro_callback(const sensor_msgs::msg::Barometer::SharedPtr msg);
    
    // EKF functions
    void Predict();
    void Update(const Eigen::VectorXd &z, Eigen::MatrixXd H);
    
    // Helper functions
    Eigen::MatrixXd H_GPS(const Eigen::VectorXd &z);
    Eigen::MatrixXd H_Baro(const Eigen::VectorXd &z);
    Eigen::MatrixXd compute_state_jacobian(double dt);
    Eigen::MatrixXd compute_input_jacobian(const Eigen::Matrix3d& rotation_matrix, double dt);
    
    // Publishing functions
    void publish_odometry();
    void publish_pose();
    void publish_twist();
    
protected:
    // ROS2 subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gps_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Barometer>::SharedPtr baro_subscriber_;
    
    // ROS2 publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_publisher_;
    
    // EKF state variables
    Eigen::VectorXd x_;  // State vector [px, py, pz, vx, vy, vz]
    Eigen::MatrixXd P_;  // Covariance matrix
    Eigen::MatrixXd Q_;  // Process noise covariance
    Eigen::MatrixXd R_;  // Measurement noise covariance
    Eigen::Quaterniond quat_;  // Orientation quaternion
    
    // Sensor data
    Eigen::Vector3d imu_accel_;
    Eigen::Vector3d imu_gyro_;
    
    // Timing
    rclcpp::Time last_time_;
    bool first_prediction_;
};

#endif // EKF_H
