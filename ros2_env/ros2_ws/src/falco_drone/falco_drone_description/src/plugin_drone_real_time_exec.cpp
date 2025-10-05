#include "falco_drone_description/plugin_drone_realtime.h"
#include <chrono>

namespace gazebo_plugins
{

DroneRealtimeController::DroneRealtimeController(const std::string& node_name)
: rclcpp::Node(node_name)
  , impl_(std::make_unique<DroneRealtimeControllerPrivate>())
{
  // Initialize the implementation
  impl_->ros_node_ = shared_from_this();
  impl_->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());

  last_update_time_ = this->now();
}

DroneRealtimeController::~DroneRealtimeController()
{
  if (update_timer_) {
    update_timer_->cancel();
  }
}

void DroneRealtimeController::Initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initializing Real-time Drone Controller");

  // Initialize subscribers for real-time operation
  impl_->InitSubscribers(
    "/real_drone/cmd_vel",
    "/real_drone/posctrl", 
    "/real_drone/takeoff",
    "/real_drone/land",
    "/real_drone/reset",
    "/real_drone/dronevel_mode",
    "/real_drone/ekf/pose",
    "/real_drone/ekf/twist"
  );

  // Initialize publishers for real-time operation
  impl_->InitPublishers(
    "/real_drone/cmd_mode",
    "/real_drone/state",
    "/real_drone/odom",
    "/real_drone/control_output"
  );

  // Enable odometry publishing
  impl_->pub_odom = true;

  // Set default controller parameters if no SDF available
  // These would normally be loaded from SDF in simulation
  impl_->mass = 1.0; // kg
  impl_->inertia = Eigen::Vector3d(0.01, 0.01, 0.01); // kg*m^2

  // Initialize PID controllers with default values
  // These should be tuned for your specific drone
  // Roll/Pitch controllers
  impl_->controllers_.roll.gain_p = 4.0;
  impl_->controllers_.roll.gain_i = 0.0;
  impl_->controllers_.roll.gain_d = 0.35;
  impl_->controllers_.roll.limit = 10.0;
  impl_->controllers_.roll.time_constant = 0.0;

  impl_->controllers_.pitch.gain_p = 4.0;
  impl_->controllers_.pitch.gain_i = 0.0;
  impl_->controllers_.pitch.gain_d = 0.35;
  impl_->controllers_.pitch.limit = 10.0;
  impl_->controllers_.pitch.time_constant = 0.0;

  // Yaw controller
  impl_->controllers_.yaw.gain_p = 2.0;
  impl_->controllers_.yaw.gain_i = 0.0;
  impl_->controllers_.yaw.gain_d = 0.0;
  impl_->controllers_.yaw.limit = 1.0;
  impl_->controllers_.yaw.time_constant = 0.0;

  // Velocity controllers
  impl_->controllers_.velocity_x.gain_p = 5.0;
  impl_->controllers_.velocity_x.gain_i = 0.0;
  impl_->controllers_.velocity_x.gain_d = 0.0;
  impl_->controllers_.velocity_x.limit = 5.0;
  impl_->controllers_.velocity_x.time_constant = 0.0;

  impl_->controllers_.velocity_y.gain_p = 5.0;
  impl_->controllers_.velocity_y.gain_i = 0.0;
  impl_->controllers_.velocity_y.gain_d = 0.0;
  impl_->controllers_.velocity_y.limit = 5.0;
  impl_->controllers_.velocity_y.time_constant = 0.0;

  impl_->controllers_.velocity_z.gain_p = 10.0;
  impl_->controllers_.velocity_z.gain_i = 2.0;
  impl_->controllers_.velocity_z.gain_d = 0.0;
  impl_->controllers_.velocity_z.limit = 10.0;
  impl_->controllers_.velocity_z.time_constant = 0.0;

  // Position controllers
  impl_->controllers_.pos_x.gain_p = 3.0;
  impl_->controllers_.pos_x.gain_i = 0.0;
  impl_->controllers_.pos_x.gain_d = 0.0;
  impl_->controllers_.pos_x.limit = 5.0;
  impl_->controllers_.pos_x.time_constant = 0.0;

  impl_->controllers_.pos_y.gain_p = 3.0;
  impl_->controllers_.pos_y.gain_i = 0.0;
  impl_->controllers_.pos_y.gain_d = 0.0;
  impl_->controllers_.pos_y.limit = 5.0;
  impl_->controllers_.pos_y.time_constant = 0.0;

  impl_->controllers_.pos_z.gain_p = 10.0;
  impl_->controllers_.pos_z.gain_i = 0.0;
  impl_->controllers_.pos_z.gain_d = 0.0;
  impl_->controllers_.pos_z.limit = 5.0;
  impl_->controllers_.pos_z.time_constant = 0.0;

  RCLCPP_INFO(this->get_logger(), "Real-time Drone Controller initialized with default PID parameters");
}

void DroneRealtimeController::Run()
{
  // Create update timer
  auto update_period = std::chrono::duration<double>(1.0 / update_rate_);
  update_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(update_period),
    std::bind(&DroneRealtimeController::UpdateLoop, this)
  );

  RCLCPP_INFO(this->get_logger(), "Real-time Drone Controller running at %.1f Hz", update_rate_);
}

void DroneRealtimeController::UpdateLoop()
{
  auto current_time = this->now();
  double dt = (current_time - last_update_time_).seconds();
  last_update_time_ = current_time;

  // Ensure reasonable dt
  if (dt > 0.1) dt = 0.01; // Limit to 100ms max
  if (dt < 0.001) dt = 0.01; // Minimum 1ms

  // Update state and dynamics
  impl_->UpdateState(dt);
  impl_->UpdateDynamics(dt);
}

}