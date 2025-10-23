#include "drone_sensors_cpp/barometer_simulator.hpp"
#include <cmath>

namespace drone_sensors_cpp
{

BarometerSimulator::BarometerSimulator()
: Node("barometer_simulator"),
  gen_(rd_()),
  noise_dist_(0.0, pressure_noise_std_),
  bias_drift_dist_(0.0, bias_drift_std_)
{
  // Publishers
  pressure_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/drone/barometer", 10);
  
  // Subscribers
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/drone/pose",
    10,
    std::bind(&BarometerSimulator::pose_callback, this, std::placeholders::_1));
  
  // Timer for sensor updates (10 Hz)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&BarometerSimulator::publish_pressure, this));
  
  RCLCPP_INFO(this->get_logger(), "Barometer Simulator initialized");
}

void BarometerSimulator::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Convert from NED to altitude above ground
  current_altitude_ = -msg->pose.position.z;
}

double BarometerSimulator::altitude_to_pressure(double altitude)
{
  // Standard atmosphere model
  double temp_at_altitude = temperature_ - lapse_rate_ * altitude;
  double temp_kelvin = temp_at_altitude + 273.15;
  double temp_sea_level_kelvin = temperature_ + 273.15;
  
  // Barometric formula
  // P = P0 * (T/T0)^(g*M/(R*L))
  // where:
  // g = 9.80665 m/s² (gravitational acceleration)
  // M = 0.0289644 kg/mol (molar mass of Earth's air)
  // R = 8.31432 J/(mol·K) (universal gas constant)
  // L = lapse_rate (temperature lapse rate)
  
  double exponent = (9.80665 * 0.0289644) / (8.31432 * lapse_rate_);
  double pressure = sea_level_pressure_ * std::pow(temp_kelvin / temp_sea_level_kelvin, exponent);
  
  return pressure;
}

void BarometerSimulator::publish_pressure()
{
  try {
    // Calculate ideal pressure
    double ideal_pressure = altitude_to_pressure(current_altitude_);
    
    // Add noise and bias
    double noise = noise_dist_(gen_);
    pressure_bias_ += bias_drift_dist_(gen_) * 0.1;  // Bias drift (scaled by time step)
    
    double measured_pressure = ideal_pressure + noise + pressure_bias_;
    
    // Publish pressure
    auto pressure_msg = std_msgs::msg::Float64();
    pressure_msg.data = measured_pressure;
    pressure_pub_->publish(pressure_msg);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Barometer simulation error: %s", e.what());
  }
}

}  // namespace drone_sensors_cpp

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_sensors_cpp::BarometerSimulator>();
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
