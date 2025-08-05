#include "BatteryMonitor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char **argv) {
  
  rclcpp::init(argc, argv);
  std::shared_ptr<BatteryMonitor>Batteryinstance = std::make_shared<BatteryMonitor>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec->add_node(Batteryinstance);
  std::jthread spin_ros([this]()
{exec->spin();});
  std::jthread Actual_Battery_Thread([this](){
    Batteryinstance->Startup();
  })
  spin_ros.join(); // Waits for the ROS to shutdown.
  Batteryinstance->Shutdown();
  rclcpp::shutdown();
  
  return 0;
}