#include <rclcpp/rclcpp.hpp>
#include "falco_drone_description/plugin_drone_realtime.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<gazebo_plugins::DroneRealtimeController>("drone_realtime_controller");
  
  try {
    controller->Initialize();
    controller->Run();
    
    RCLCPP_INFO(controller->get_logger(), "Starting Real-time Drone Controller");
    rclcpp::spin(controller);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(controller->get_logger(), "Exception in drone controller: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}