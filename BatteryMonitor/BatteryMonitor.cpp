#include "BatteryMonitor.hpp"
#include <chrono>
#include <functional>
#include <memory>
#define DANGERSOCLEVEL 5

/// @brief Step Function that also generates SOC INT.
double BatteryMonitor::CalcSOC() {
  double resultingSOC;
  //  wait and set value for current value and Time Final.
   // get READING;
  timeFinal = std::chrono::steady_clock::now();
  auto deltaTime =
      std::chrono::duration<double>(timeFinal - timeInital).count();
  resultingSOC =
      previousSOC + (CurrentCurrent / currentRating) * (deltaTime);

  // Setup for next Reading of SOC
  previousSOC = resultingSOC;
  timeInital = timeFinal;

  if (resultingSOC <= DANGERSOCLEVEL) {
    auto msg = std::make_unique<std_msgs::msg::Bool>();
    msg->data = true;
    SOCINTPublisher->publish(std::move(msg));
  }
  return resultingSOC;
}
/// @brief Callback ROS2 topic function
void BatteryMonitor::getVolt(const std_msgs::msg::Float64::SharedPtr msg) {
  CurrentVolt = static_cast<double>(msg->data);
}
/// @brief Callback ROS2 topic function. Get Current should be negative when the robot is in the pool.
void BatteryMonitor::getCurrent(const std_msgs::msg::Float64::SharedPtr msg) {
  CurrentCurrent = static_cast<double>(msg->data);
}
void BatteryMonitor::SOC_RealTime_callback(){
  auto msg = std::make_unique<std_msgs::msg::Float64>();
  msg->data = static_cast<_Float64>(CalcSOC());
  SOCPublisher->publish(std::move(msg));
}