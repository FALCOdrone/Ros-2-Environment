#ifndef DRONE_SENSORS_CPP__BAROMETER_SIMULATOR_HPP_
#define DRONE_SENSORS_CPP__BAROMETER_SIMULATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <random>
#include <cmath>

namespace drone_sensors_cpp
{

class BarometerSimulator : public rclcpp::Node
{
public:
  BarometerSimulator();

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publish_pressure();
  double altitude_to_pressure(double altitude);
  
  // Barometer parameters
  double sea_level_pressure_{1013.25};  // hPa
  double temperature_{15.0};            // Celsius at sea level
  double lapse_rate_{0.0065};           // K/m
  
  // Noise parameters
  double pressure_noise_std_{0.1};      // hPa
  double bias_drift_std_{0.01};         // hPa/s
  double pressure_bias_{0.0};
  
  // Current altitude
  double current_altitude_{0.0};
  
  // Publishers and subscribers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pressure_pub_;
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

#endif  // DRONE_SENSORS_CPP__BAROMETER_SIMULATOR_HPP_
