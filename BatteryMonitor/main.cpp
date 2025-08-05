#include "BatteryMonitor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  std::shared_ptr<BatteryMonitor> Batteryinstance = std::make_shared<BatteryMonitor>();
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exec->add_node(Batteryinstance);
  std::jthread spin_ros([exec]()
                        { exec->spin(); });
  std::jthread Actual_Battery_Thread([Batteryinstance]()
                                     { Batteryinstance->Startup(); });

  spin_ros.join(); // Waits for the ROS to shutdown.
  Batteryinstance->Shutdown();
  rclcpp::shutdown();

  return 0;
}