#ifndef GAZEBO_PLUGINS_DRONE_REALTIME_H
#define GAZEBO_PLUGINS_DRONE_REALTIME_H

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>

#include "falco_drone_description/plugin_drone_realtime_private.h"

namespace gazebo_plugins
{

class DroneRealtimeController : public rclcpp::Node
{
public:
  DroneRealtimeController(const std::string& node_name);
  virtual ~DroneRealtimeController();

  void Initialize();
  void Run();
  void UpdateLoop();

private:
  std::unique_ptr<DroneRealtimeControllerPrivate> impl_;
  
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Time last_update_time_;
  double update_rate_ = 100.0; // Hz
};

} // namespace gazebo_plugins

#endif // GAZEBO_PLUGINS_DRONE_REALTIME_H
