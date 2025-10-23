#ifndef DRONE_SENSORS_CPP__MAGNETOMETER_SIMULATOR_HPP_
#define DRONE_SENSORS_CPP__MAGNETOMETER_SIMULATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <array>
#include <random>

namespace drone_sensors_cpp
{

class MagnetometerSimulator : public rclcpp::Node
{
public:
  MagnetometerSimulator();

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publish_magnetometer();
  std::array<double, 3> quaternion_to_body_magnetic_field(const tf2::Quaternion& q);
  
  // Earth's magnetic field parameters (typical values)
  // Magnetic field in NED frame (North, East, Down)
  std::array<double, 3> earth_magnetic_field_{0.2, 0.0, 0.4};  // Gauss
  
  // Noise parameters
  double mag_noise_std_{0.01};    // Gauss
  double bias_drift_std_{0.001};  // Gauss/s
  std::array<double, 3> mag_bias_{0.0, 0.0, 0.0};
  
  // Current attitude
  tf2::Quaternion current_orientation_{0, 0, 0, 1};  // [x,y,z,w]
  
  // Publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  
  // Timer for sensor updates
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Random number generation
  std::random_device rd_;
  std::mt19937 gen_;
  std::normal_distribution<double> noise_dist_;
  std::normal_distribution<double> bias_drift_dist_;
};

}  // namespace drone_sensors_cpp

#endif  // DRONE_SENSORS_CPP__MAGNETOMETER_SIMULATOR_HPP_
