#include "BatteryMonitor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatteryMonitor>();
  rclcpp::spin(node);
  std::thread StartupThread(&BatteryMonitor::Startup, &(*node));
  StartupThread.detach();
  std::cout << "Finished" << std::endl;
  rclcpp::shutdown();
  return 0;
}