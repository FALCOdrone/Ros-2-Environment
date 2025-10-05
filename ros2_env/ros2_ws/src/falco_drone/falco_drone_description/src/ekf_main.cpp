#include "falco_drone_description/ekf.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create EKF node
    auto ekf_node = std::make_shared<EKF>();
    
    // Initialize EKF with initial state and covariance
    Eigen::VectorXd x0(6);
    x0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  // Initial position and velocity
    
    Eigen::MatrixXd P0(6, 6);
    P0 = Eigen::MatrixXd::Identity(6, 6) * 1.0;  // Initial uncertainty
    
    Eigen::VectorXd quat0(4);
    quat0 << 0.0, 0.0, 0.0, 1.0;  // Initial orientation (identity quaternion)
    
    ekf_node->Init(x0, P0, quat0);
    
    // Spin the node
    rclcpp::spin(ekf_node); // Keep the node alive until shutdown is requested -> this allows the EKF to process incoming IMU and GPS data
    
    rclcpp::shutdown();
    return 0;
}