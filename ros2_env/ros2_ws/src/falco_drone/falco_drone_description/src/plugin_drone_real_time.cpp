#include "falco_drone_description/plugin_drone_realtime_private.h"
#include <geometry_msgs/msg/wrench.hpp>

namespace gazebo_plugins
{

DroneRealtimeControllerPrivate::DroneRealtimeControllerPrivate()
: m_timeAfterCmd(0.0)
  , navi_state(LANDED_MODEL)
  , m_posCtrl(false)
  , m_velMode(false)
  , odom_seq(0)
  , odom_hz(30)
  , last_odom_publish_time_(0.0)
  , mass(1.0)
{
  inertia = Eigen::Vector3d(0.01, 0.01, 0.01); // TODO: set proper inertia and place the same values into the urdf and sdf file
}

DroneRealtimeControllerPrivate::~DroneRealtimeControllerPrivate() {}

void DroneRealtimeControllerPrivate::Reset()
{
  // Reset the values of the controllers
  controllers_.roll.reset();
  controllers_.pitch.reset();
  controllers_.yaw.reset();
  controllers_.velocity_x.reset();
  controllers_.velocity_y.reset();
  controllers_.velocity_z.reset();
  controllers_.pos_x.reset();
  controllers_.pos_y.reset();
  controllers_.pos_z.reset();

  // Reset the state of the drone
  position = Eigen::Vector3d::Zero();
  orientation = Eigen::Quaterniond::Identity();
  velocity = Eigen::Vector3d::Zero();
  angular_velocity = Eigen::Vector3d::Zero();
  acceleration = Eigen::Vector3d::Zero();
  euler = Eigen::Vector3d::Zero();
}

//--------------------------------------------------------
// Callbacks
//--------------------------------------------------------

void DroneRealtimeControllerPrivate::InitSubscribers(
  std::string cmd_normal_topic_,
  std::string posctrl_topic_,
  std::string takeoff_topic_,
  std::string land_topic_,
  std::string reset_topic_,
  std::string switch_mode_topic_,   
  std::string ekf_topic_pose,
  std::string ekf_topic_twist)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  if (!cmd_normal_topic_.empty()) {
    cmd_subscriber_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_normal_topic_, qos,
      std::bind(&DroneRealtimeControllerPrivate::CmdCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No cmd_topic defined!");
  }

  if (!posctrl_topic_.empty()) {
    posctrl_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
      posctrl_topic_, qos,
      std::bind(&DroneRealtimeControllerPrivate::PosCtrlCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No position control defined!");
  }

  // subscribe command: take off command
  if (!takeoff_topic_.empty()) {
    takeoff_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Empty>(
      takeoff_topic_, qos,
      std::bind(&DroneRealtimeControllerPrivate::TakeoffCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No takeoff topic defined!");
  }

  // subscribe command: land command
  if (!land_topic_.empty()) {
    land_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Empty>(
      land_topic_, qos,
      std::bind(&DroneRealtimeControllerPrivate::LandCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No land topic defined!");
  }

  // subscribe command: reset command
  if (!reset_topic_.empty()) {
    reset_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Empty>(
      reset_topic_, qos,
      std::bind(&DroneRealtimeControllerPrivate::ResetCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No reset topic defined!");
  }

  // Subscribe command: switch mode command
  if (!switch_mode_topic_.empty()) {
    switch_mode_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
      switch_mode_topic_, qos,
      std::bind(&DroneRealtimeControllerPrivate::SwitchModeCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No switch mode topic defined!");
  }

  // Subscribe to EKF pose and twist topics
  if (!ekf_topic_pose.empty()) {
    ekf_pose_subscriber_ = ros_node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      ekf_topic_pose, qos,
      std::bind(&DroneRealtimeControllerPrivate::EkfPoseCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No EKF pose topic defined!");
  }

  if (!ekf_topic_twist.empty()) {
    ekf_twist_subscriber_ = ros_node_->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      ekf_topic_twist, qos,
      std::bind(&DroneRealtimeControllerPrivate::EkfTwistCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No EKF twist topic defined!");
  }
}

void DroneRealtimeControllerPrivate::InitPublishers(
  std::string cmd_mode_topic_,
  std::string state_topic_,
  std::string odom_topic_,
  std::string control_output_topic_)
{

  if (!cmd_mode_topic_.empty()) {
    pub_cmd_mode = ros_node_->create_publisher<std_msgs::msg::String>(cmd_mode_topic_, 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No command mode topic defined!");
  }

  if (!state_topic_.empty()) {
    pub_state = ros_node_->create_publisher<std_msgs::msg::Int8>(state_topic_, 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No state topic defined!");
  }

  if (!odom_topic_.empty()) {
    pub_odom_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No odom topic defined!");
  }

  if (!control_output_topic_.empty()) {
    pub_control_output_ = ros_node_->create_publisher<geometry_msgs::msg::Wrench>(control_output_topic_, 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No control output topic defined!");
  }
}

// Controller configuration
void DroneRealtimeControllerPrivate::LoadControllerSettings(sdf::ElementPtr _sdf)
{
  // Load PID parameters from SDF
  controllers_.roll.Load(_sdf, "rollpitch");
  controllers_.pitch.Load(_sdf, "rollpitch");
  controllers_.yaw.Load(_sdf, "yaw");
  controllers_.velocity_x.Load(_sdf, "velocityXY");
  controllers_.velocity_y.Load(_sdf, "velocityXY");
  controllers_.velocity_z.Load(_sdf, "velocityZ");

  controllers_.pos_x.Load(_sdf, "positionXY");
  controllers_.pos_y.Load(_sdf, "positionXY");
  controllers_.pos_z.Load(_sdf, "positionZ");

  RCLCPP_INFO_STREAM(
    ros_node_->get_logger(), "Real-time Controller using PID parameters: \n" <<
      "\tRoll Pitch:\n" << "\t\tkP: " << controllers_.roll.gain_p << ", kI: " << controllers_.roll.gain_i << ",kD: " << controllers_.roll.gain_d << ", Limit: " << controllers_.roll.limit << ", Time Constant: " << controllers_.roll.time_constant << "\n" <<
      "\tYaw:\n" << "\t\tkP: " << controllers_.yaw.gain_p << ", kI: " << controllers_.yaw.gain_i << ",kD: " << controllers_.yaw.gain_d << ", Limit: " << controllers_.yaw.limit << ", Time Constant: " << controllers_.yaw.time_constant << "\n" <<
      "\tVelocity X:\n" << "\t\tkP: " << controllers_.velocity_x.gain_p << ", kI: " << controllers_.velocity_x.gain_i << ",kD: " << controllers_.velocity_x.gain_d << ", Limit: " << controllers_.velocity_x.limit << ", Time Constant: " << controllers_.velocity_x.time_constant << "\n" <<
      "\tVelocity Y:\n" << "\t\tkP: " << controllers_.velocity_y.gain_p << ", kI: " << controllers_.velocity_y.gain_i << ",kD: " << controllers_.velocity_y.gain_d << ", Limit: " << controllers_.velocity_y.limit << ", Time Constant: " << controllers_.velocity_y.time_constant << "\n" <<
      "\tVelocity Z:\n" << "\t\tkP: " << controllers_.velocity_z.gain_p << ", kI: " << controllers_.velocity_z.gain_i << ",kD: " << controllers_.velocity_z.gain_d << ", Limit: " << controllers_.velocity_z.limit << ", Time Constant: " << controllers_.velocity_z.time_constant << "\n" <<
      "\tPosition XY:\n" << "\t\tkP: " << controllers_.pos_x.gain_p << ", kI: " << controllers_.pos_x.gain_i << ",kD: " << controllers_.pos_x.gain_d << ", Limit: " << controllers_.pos_x.limit << ", Time Constant: " << controllers_.pos_x.time_constant << "\n" <<
      "\tPosition Z:\n" << "\t\tkP: " << controllers_.pos_z.gain_p << ", kI: " << controllers_.pos_z.gain_i << ",kD: " << controllers_.pos_z.gain_d << ", Limit: " << controllers_.pos_z.limit << ", Time Constant: " << controllers_.pos_z.time_constant
  );
}

//--------------------------------------------------------
// Callbacks
//--------------------------------------------------------

void DroneRealtimeControllerPrivate::CmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
{
  cmd_vel = *cmd;
  // No noise generation for real-time mode
}

void DroneRealtimeControllerPrivate::PosCtrlCallback(const std_msgs::msg::Bool::SharedPtr cmd)
{
  m_posCtrl = cmd->data;
}

void DroneRealtimeControllerPrivate::TakeoffCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  if (navi_state == LANDED_MODEL) {
    navi_state = TAKINGOFF_MODEL;
    m_timeAfterCmd = 0;
    RCLCPP_INFO(ros_node_->get_logger(), "Real-time Quadrotor takes off!!");
  }
}

void DroneRealtimeControllerPrivate::LandCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  if (navi_state == FLYING_MODEL || navi_state == TAKINGOFF_MODEL) {
    navi_state = LANDING_MODEL;
    m_timeAfterCmd = 0;
    RCLCPP_INFO(ros_node_->get_logger(), "Real-time Quadrotor lands!!");
  }
}

void DroneRealtimeControllerPrivate::ResetCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  RCLCPP_INFO(ros_node_->get_logger(), "Reset real-time quadrotor!!");
  Reset();
}

void DroneRealtimeControllerPrivate::EkfPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  latest_ekf_pose_ = msg;
}

void DroneRealtimeControllerPrivate::EkfTwistCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  latest_ekf_twist_ = msg;
}

void DroneRealtimeControllerPrivate::SwitchModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  m_velMode = msg->data;

  std_msgs::msg::String mode;
  if (m_velMode) {
    mode.data = "velocity";
  } else {
    mode.data = "position";
  }
  pub_cmd_mode->publish(mode);
}

//---------------------------------------------------------
// Publishers
//---------------------------------------------------------

void DroneRealtimeControllerPrivate::PublishControlOutput(const Eigen::Vector3d& force, const Eigen::Vector3d& torque)
{
  geometry_msgs::msg::Wrench wrench_msg;
  wrench_msg.force.x = force.x();
  wrench_msg.force.y = force.y();
  wrench_msg.force.z = force.z();
  wrench_msg.torque.x = torque.x();
  wrench_msg.torque.y = torque.y();
  wrench_msg.torque.z = torque.z();
  
  pub_control_output_->publish(wrench_msg);
}

void DroneRealtimeControllerPrivate::PublishOdom(
  const Eigen::Vector3d & position,
  const Eigen::Quaterniond & orientation,
  const Eigen::Vector3d & velocity,
  const Eigen::Vector3d & acceleration)
{
  // Prepare the Odometry message
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = ros_node_->now();
  std::string ns = ros_node_->get_namespace();
  odom.header.frame_id = ns + "/odom";
  odom.child_frame_id = ns + "/base_footprint";

  // Set position and orientation
  odom.pose.pose.position.x = position.x();
  odom.pose.pose.position.y = position.y();
  odom.pose.pose.position.z = position.z();
  odom.pose.pose.orientation.w = orientation.w();
  odom.pose.pose.orientation.x = orientation.x();
  odom.pose.pose.orientation.y = orientation.y();
  odom.pose.pose.orientation.z = orientation.z();

  // Convert velocities to child frame (aka base_footprint)
  Eigen::Vector3d linear_child = orientation.inverse() * velocity;
  Eigen::Vector3d angular_child = orientation.inverse() * angular_velocity;
  
  odom.twist.twist.linear.x = linear_child.x();
  odom.twist.twist.linear.y = linear_child.y();
  odom.twist.twist.linear.z = linear_child.z();
  odom.twist.twist.angular.x = angular_child.x();
  odom.twist.twist.angular.y = angular_child.y();
  odom.twist.twist.angular.z = angular_child.z();

  // Publish the odometry message with coordinates relative to the base_footprint
  pub_odom_->publish(odom);

  // Publish the TF transformation
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = odom.header.stamp;
  transformStamped.header.frame_id = ns + "/odom";
  transformStamped.child_frame_id = ns + "/base_footprint";
  transformStamped.transform.translation.x = odom.pose.pose.position.x;
  transformStamped.transform.translation.y = odom.pose.pose.position.y;
  transformStamped.transform.translation.z = odom.pose.pose.position.z;
  transformStamped.transform.rotation = odom.pose.pose.orientation;

  tf_broadcaster_->sendTransform(transformStamped);
}

void DroneRealtimeControllerPrivate::UpdateState(double dt)
{
  if (navi_state == TAKINGOFF_MODEL) {
    m_timeAfterCmd += dt;
    if (m_timeAfterCmd > 0.5) {
      navi_state = FLYING_MODEL;
      std::cout << "Real-time: Entering flying model!" << std::endl;
    }
  } else if (navi_state == LANDING_MODEL) {
    m_timeAfterCmd += dt;
    if (m_timeAfterCmd > 1.0) {
      navi_state = LANDED_MODEL;
      std::cout << "Real-time: Landed!" << std::endl;
    }
  } else {
    m_timeAfterCmd = 0;
  }

  // publish current state using pub_state
  std_msgs::msg::Int8 state_msg;
  state_msg.data = navi_state;
  pub_state->publish(state_msg);
}

void DroneRealtimeControllerPrivate::UpdateDynamics(double dt)
{
  // Get the ekf position and velocity from the subscriber ekf topic
  if (!latest_ekf_pose_ || !latest_ekf_twist_) {
    RCLCPP_ERROR(ros_node_->get_logger(), "EKF pose or twist data not available!");
    return;
  }

  Eigen::Vector3d world_position(
    latest_ekf_pose_->pose.pose.position.x,
    latest_ekf_pose_->pose.pose.position.y,
    latest_ekf_pose_->pose.pose.position.z);
  
  Eigen::Vector3d world_velocity(
    latest_ekf_twist_->twist.twist.linear.x,
    latest_ekf_twist_->twist.twist.linear.y,
    latest_ekf_twist_->twist.twist.linear.z);

  // orientation from ekf (which is directly from imu)
  Eigen::Quaterniond ekf_orientation(
    latest_ekf_pose_->pose.pose.orientation.w,
    latest_ekf_pose_->pose.pose.orientation.x,
    latest_ekf_pose_->pose.pose.orientation.y,
    latest_ekf_pose_->pose.pose.orientation.z);

  // Compute the filtered acceleration from the world velocity
  double current_time = rclcpp::Clock().now().seconds();
  double dt_real = current_time - previous_time;
  previous_time = current_time;

  Eigen::Vector3d world_acceleration = Eigen::Vector3d::Zero();
  if (dt_real > 0.0) {
    world_acceleration = (world_velocity - prev_world_velocity) / dt_real;
  }
  prev_world_velocity = world_velocity;

  // Update state variables
  position = world_position;
  orientation = ekf_orientation;
  velocity = world_velocity;
  acceleration = world_acceleration;
  angular_velocity = Eigen::Vector3d(
    latest_ekf_twist_->twist.twist.angular.x,
    latest_ekf_twist_->twist.twist.angular.y,
    latest_ekf_twist_->twist.twist.angular.z);
  euler = orientation.toRotationMatrix().eulerAngles(0, 1, 2); // Convert quaternion to Euler angles

  //convert the acceleration and velocity into the body frame
  Eigen::Vector3d body_vel = orientation.inverse() * velocity;
  Eigen::Vector3d body_acc = orientation.inverse() * acceleration;

  Eigen::Vector3d poschange = position - prev_position;
  prev_position = position;

  // Get gravity and setup control variables
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  Eigen::Vector3d torque = Eigen::Vector3d::Zero();
  double gravity = 9.81; // Standard gravity
  double load_factor = 1.0; // Load factor for real drone, can be adjusted based on payload
  // if load_factor > 1.0, it means that we are applying more force than just the gravity to compensate for additional load
  // TODO: implement a more flexible load compensation mechanism, e.g. through an estimation of the constant load force (exstending the ekf state)

  // Create heading quaternion for control frame transformation
  double yaw = euler.z();
  Eigen::Quaterniond heading_quaternion(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d velocity_xy = heading_quaternion.inverse() * velocity; // body frame velocity
  Eigen::Vector3d acceleration_xy = heading_quaternion.inverse() * acceleration; // body frame acceleration
  Eigen::Vector3d angular_velocity_body = orientation.inverse() * angular_velocity;

  if (m_posCtrl) {
    //position control
    if (navi_state == FLYING_MODEL) {
      double vx = controllers_.pos_x.update(cmd_vel.linear.x, position.x(), poschange.x(), dt);
      double vy = controllers_.pos_y.update(cmd_vel.linear.y, position.y(), poschange.y(), dt);
      double vz = controllers_.pos_z.update(cmd_vel.linear.z, position.z(), poschange.z(), dt);

      // Transform velocity commands to body frame
      Eigen::Vector3d vb = heading_quaternion.inverse() * Eigen::Vector3d(vx, vy, vz);

      double pitch_command = controllers_.velocity_x.update(
        vb.x(), velocity_xy.x(), acceleration_xy.x(), dt) / gravity;
      double roll_command = -controllers_.velocity_y.update(
        vb.y(), velocity_xy.y(), acceleration_xy.y(), dt) / gravity;

      // set a more realistic torque modeling, since we are not considering the aerodynamics dissipative terms
      torque.x() = inertia.x() * controllers_.roll.update(
        roll_command, euler.x(), angular_velocity_body.x(), dt);
      torque.y() = inertia.y() * controllers_.pitch.update(
        pitch_command, euler.y(), angular_velocity_body.y(), dt);
      force.z() = mass * (controllers_.velocity_z.update(
          vz, velocity.z(), acceleration.z(), dt) + load_factor * gravity);
    }
  } else {
    //normal control
    if (navi_state == FLYING_MODEL) {
      //hovering
      double pitch_command = controllers_.velocity_x.update(
        cmd_vel.linear.x, velocity_xy.x(), acceleration_xy.x(), dt) / gravity;
      double roll_command = -controllers_.velocity_y.update(
        cmd_vel.linear.y, velocity_xy.y(), acceleration_xy.y(), dt) / gravity;
      
      torque.x() = inertia.x() * controllers_.roll.update(
        roll_command, euler.x(), angular_velocity_body.x(), dt);
      torque.y() = inertia.y() * controllers_.pitch.update(
        pitch_command, euler.y(), angular_velocity_body.y(), dt);
    } else {
      //control by velocity
      if (m_velMode) {
        double pitch_command = controllers_.velocity_x.update(
          cmd_vel.angular.x, velocity_xy.x(), acceleration_xy.x(), dt) / gravity;
        double roll_command = -controllers_.velocity_y.update(
          cmd_vel.angular.y, velocity_xy.y(), acceleration_xy.y(), dt) / gravity;
        
        torque.x() = inertia.x() * controllers_.roll.update(
          roll_command, euler.x(), angular_velocity_body.x(), dt);
        torque.y() = inertia.y() * controllers_.pitch.update(
          pitch_command, euler.y(), angular_velocity_body.y(), dt);
      } else {
        //control by tilting
        torque.x() = inertia.x() * controllers_.roll.update(
          cmd_vel.angular.x, euler.x(), angular_velocity_body.x(), dt);
        torque.y() = inertia.y() * controllers_.pitch.update(
          cmd_vel.angular.y, euler.y(), angular_velocity_body.y(), dt);
      }
    }
    torque.z() = inertia.z() * controllers_.yaw.update(cmd_vel.angular.z, angular_velocity.z(), 0, dt);
    force.z() = mass * (controllers_.velocity_z.update(
        cmd_vel.linear.z, velocity.z(), acceleration.z(), dt) + load_factor * gravity);
  }

  // Limit maximum force
  if (force.z() > 20.0) {force.z() = 20.0;}  // Reasonable max force for real drone
  if (force.z() < 0.0) {force.z() = 0.0;}

  // limit maximum torque -> check values from datasheet of real drone
  if (torque.x() > 1.0) {torque.x() = 1.0;}
  if (torque.x() < -1.0) {torque.x() = -1.0;}
  if (torque.y() > 1.0) {torque.y() = 1.0;}
  if (torque.y() < -1.0) {torque.y() = -1.0;}
  if (torque.z() > 1.0) {torque.z() = 1.0;}
  if (torque.z() < -1.0) {torque.z() = -1.0;}

  // Publish control output for real drone
  if (navi_state != LANDED_MODEL) {
    PublishControlOutput(force, torque);
    RCLCPP_DEBUG(ros_node_->get_logger(), 
      "Real-time Control Output: Force=[%.3f, %.3f, %.3f], Torque=[%.3f, %.3f, %.3f]",
      force.x(), force.y(), force.z(), torque.x(), torque.y(), torque.z());
  }

  if (pub_odom) {
    double current_time_double = current_time;
    if (current_time_double - last_odom_publish_time_ >= 1.0 / odom_hz) {
      PublishOdom(position, orientation, velocity, acceleration);
      last_odom_publish_time_ = current_time_double;
      odom_seq++;
    }
  }
}

} // namespace gazebo_plugins