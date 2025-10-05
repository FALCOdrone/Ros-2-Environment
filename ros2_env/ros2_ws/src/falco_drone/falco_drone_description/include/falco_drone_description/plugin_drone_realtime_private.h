#ifndef GAZEBO_PLUGINS_DRONE_REALTIME_PRIVATE_H
#define GAZEBO_PLUGINS_DRONE_REALTIME_PRIVATE_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <sdf/sdf.hh>

#include "falco_drone_description/pid_controller.h"

namespace gazebo_plugins
{

// States
enum DroneState {
  LANDED_MODEL = 0,
  FLYING_MODEL = 1,
  TAKINGOFF_MODEL = 2,
  LANDING_MODEL = 3
};

struct Controllers {
  PidController roll;
  PidController pitch;
  PidController yaw;
  PidController velocity_x;
  PidController velocity_y;
  PidController velocity_z;
  PidController pos_x;
  PidController pos_y;
  PidController pos_z;
};

class DroneRealtimeControllerPrivate
{
public:
  DroneRealtimeControllerPrivate();
  virtual ~DroneRealtimeControllerPrivate();

  void Reset();
  
  // Initialization methods
  void InitSubscribers(
    std::string cmd_normal_topic_,
    std::string posctrl_topic_,
    std::string takeoff_topic_,
    std::string land_topic_,
    std::string reset_topic_,
    std::string switch_mode_topic_,
    std::string ekf_topic_pose,
    std::string ekf_topic_twist);
    
  void InitPublishers(
    std::string cmd_mode_topic_,
    std::string state_topic_,
    std::string odom_topic_,
    std::string control_output_topic_);
    
  void LoadControllerSettings(sdf::ElementPtr _sdf);

  // Callback methods
  void CmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd);
  void PosCtrlCallback(const std_msgs::msg::Bool::SharedPtr cmd);
  void TakeoffCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void LandCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void ResetCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void SwitchModeCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void EkfPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void EkfTwistCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

  // Publisher methods
  void PublishControlOutput(const Eigen::Vector3d& force, const Eigen::Vector3d& torque);
  void PublishOdom(
    const Eigen::Vector3d & position,
    const Eigen::Quaterniond & orientation,
    const Eigen::Vector3d & velocity,
    const Eigen::Vector3d & acceleration);

  // Update methods
  void UpdateState(double dt);
  void UpdateDynamics(double dt);

  // ROS node and communication
  rclcpp::Node::SharedPtr ros_node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr posctrl_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr takeoff_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr land_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr switch_mode_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pose_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr ekf_twist_subscriber_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cmd_mode;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_state;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_control_output_;

  // State variables
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d euler;
  Eigen::Vector3d prev_position;
  Eigen::Vector3d prev_world_velocity;

  // EKF data
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_ekf_pose_;
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr latest_ekf_twist_;

  // Control variables
  geometry_msgs::msg::Twist cmd_vel;
  Controllers controllers_;
  
  // Drone parameters
  double mass;
  Eigen::Vector3d inertia;
  
  // State management
  double m_timeAfterCmd;
  int navi_state;
  bool m_posCtrl;
  bool m_velMode;
  bool pub_odom;
  
  // Odometry
  int odom_seq;
  double odom_hz;
  double last_odom_publish_time_;
  double previous_time;
};

} // namespace gazebo_plugins

#endif // GAZEBO_PLUGINS_DRONE_REALTIME_PRIVATE_H
