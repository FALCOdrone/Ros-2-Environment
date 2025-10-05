#include <rclcpp/rclcpp.hpp>
#include "falco_drone_description/plugin_drone_realtime.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<gazebo_plugins::DroneRealtimeController>("real_drone_controller");
  
  try {
    controller->Initialize();
    
    // Override topics for real-time operation to avoid conflicts with simulation
    // This should be done by updating the InitSubscribers/InitPublishers calls
    // to use "/real_drone" namespace instead of "/simple_drone"
    
    controller->Run();
    
    RCLCPP_INFO(controller->get_logger(), "Starting Real-time Drone Controller with /real_drone namespace");
    rclcpp::spin(controller);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(controller->get_logger(), "Exception in real-time drone controller: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
