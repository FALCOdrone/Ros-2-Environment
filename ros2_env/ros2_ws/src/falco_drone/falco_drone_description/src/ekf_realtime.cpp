#include "falco_drone_description/ekf.h"
#include <rclcpp/rclcpp.hpp>

class EKFRealtime : public EKF {
public:
    EKFRealtime() : EKF() {
        // Override topic names for real-time operation
        std::string imu_topic = "/real_drone/imu/out";
        std::string gps_topic = "/real_drone/gps_position/out";
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        
        // Re-create subscribers with real-time topics
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, qos,
            std::bind(&EKF::imu_callback, this, std::placeholders::_1));

        gps_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            gps_topic, qos,
            std::bind(&EKF::gps_callback, this, std::placeholders::_1));

        baro_subscriber_ = this->create_subscription<sensor_msgs::msg::Barometer>(
            "/real_drone/barometer/out", qos,
            std::bind(&EKF::baro_callback, this, std::placeholders::_1));
        
        // Re-create publishers with real-time namespace
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/real_drone/ekf/odometry", 10);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/real_drone/ekf/pose", 10);
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/real_drone/ekf/twist", 10);

        RCLCPP_INFO(this->get_logger(), "Real-time EKF Node initialized");
    }
};

int main(int argc, char **argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create Real-time EKF node
    auto ekf_node = std::make_shared<EKFRealtime>();
    
    // Initialize EKF with initial state and covariance
    Eigen::VectorXd x0(7);
    x0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  // Initial position, velocity, and yaw_rate

    Eigen::MatrixXd P0(7, 7);
    P0 = Eigen::MatrixXd::Identity(7, 7) * 1.0;  // Initial uncertainty

    Eigen::VectorXd quat0(4);
    quat0 << 0.0, 0.0, 0.0, 1.0;  // Initial orientation (identity quaternion)
    
    ekf_node->Init(x0, P0, quat0);
    
    RCLCPP_INFO(ekf_node->get_logger(), "Starting Real-time EKF for hardware integration");
    
    // Spin the node
    rclcpp::spin(ekf_node);
    
    rclcpp::shutdown();
    return 0;
}
