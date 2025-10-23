#include "drone_sensors_cpp/magnetometer_simulator.hpp"
#include <cmath>

namespace drone_sensors_cpp
{

MagnetometerSimulator::MagnetometerSimulator()
: Node("magnetometer_simulator"), 
  gen_(rd_()),
  noise_dist_(0.0, mag_noise_std_),
  bias_drift_dist_(0.0, bias_drift_std_)
{
  // Publishers
  mag_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/drone/magnetometer", 10);
  
  // Subscribers
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/drone/pose",
    10,
    std::bind(&MagnetometerSimulator::pose_callback, this, std::placeholders::_1));
  
  // Timer for sensor updates (20 Hz)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&MagnetometerSimulator::publish_magnetometer, this));
  
  RCLCPP_INFO(this->get_logger(), "Magnetometer Simulator initialized");
}

void MagnetometerSimulator::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Update current orientation from pose
  current_orientation_.setX(msg->pose.orientation.x);
  current_orientation_.setY(msg->pose.orientation.y);
  current_orientation_.setZ(msg->pose.orientation.z);
  current_orientation_.setW(msg->pose.orientation.w);
}

std::array<double, 3> MagnetometerSimulator::quaternion_to_body_magnetic_field(const tf2::Quaternion& q)
{
  // Convert quaternion to rotation matrix
  tf2::Matrix3x3 rotation_matrix(q);
  
  // Transform earth's magnetic field from NED to body frame
  // R_ned_to_body = R_body_to_ned^T (transpose of rotation matrix)
  std::array<double, 3> mag_body;
  
  // Manual matrix multiplication: mag_body = R_ned_to_body * earth_magnetic_field
  // Since we want NED to body, we use the transpose (inverse) of the rotation matrix
  mag_body[0] = rotation_matrix[0][0] * earth_magnetic_field_[0] + 
                rotation_matrix[1][0] * earth_magnetic_field_[1] + 
                rotation_matrix[2][0] * earth_magnetic_field_[2];
                
  mag_body[1] = rotation_matrix[0][1] * earth_magnetic_field_[0] + 
                rotation_matrix[1][1] * earth_magnetic_field_[1] + 
                rotation_matrix[2][1] * earth_magnetic_field_[2];
                
  mag_body[2] = rotation_matrix[0][2] * earth_magnetic_field_[0] + 
                rotation_matrix[1][2] * earth_magnetic_field_[1] + 
                rotation_matrix[2][2] * earth_magnetic_field_[2];
  
  return mag_body;
}

void MagnetometerSimulator::publish_magnetometer()
{
  try {
    // Transform earth's magnetic field to body frame
    auto mag_body = quaternion_to_body_magnetic_field(current_orientation_);
    
    // Add noise and bias
    std::array<double, 3> noise;
    for (size_t i = 0; i < 3; ++i) {
      noise[i] = noise_dist_(gen_);
      mag_bias_[i] += bias_drift_dist_(gen_) * 0.05;  // Bias drift (scaled by time step)
    }
    
    std::array<double, 3> measured_mag;
    for (size_t i = 0; i < 3; ++i) {
      measured_mag[i] = mag_body[i] + noise[i] + mag_bias_[i];
    }
    
    // Publish magnetometer reading
    auto mag_msg = geometry_msgs::msg::Vector3Stamped();
    mag_msg.header.stamp = this->get_clock()->now();
    mag_msg.header.frame_id = "magnetometer_link";
    
    mag_msg.vector.x = measured_mag[0];
    mag_msg.vector.y = measured_mag[1];
    mag_msg.vector.z = measured_mag[2];
    
    mag_pub_->publish(mag_msg);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Magnetometer simulation error: %s", e.what());
  }
}

}  // namespace drone_sensors_cpp

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_sensors_cpp::MagnetometerSimulator>();
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
