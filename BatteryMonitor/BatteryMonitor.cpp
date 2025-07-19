#include "BatteryMonitor.h"
#include <chrono>
#define DANGERSOCLEVEL 5

/// @brief Step Function that also generates SOC INT.
double BatteryMonitor::CalcSOC() {
  double resultingSOC;
  //  wait and set value for current value and Time Final.
  currentCurrentValue = getCurrent(); // get READING;
  timeFinal = std::chrono::steady_clock::now();
  auto deltaTime =
      std::chrono::duration<double>(timeFinal - timeInital).count();
  resultingSOC =
      previousSOC + (currentCurrentValue / currentRating) * (deltaTime);

  // Setup for next Reading of SOC
  previousSOC = resultingSOC;
  timeInital = timeFinal;

  if (resultingSOC <= DANGERSOCLEVEL) {
    SOCINTPublisher->publish(true);
  }
  return resultingSOC;
}
/// @brief Callback ROS2 topic function
double BatteryMonitor::ReadVolt() {
  double resultVolt;

  return resultVolt;
}
/// @brief Callback ROS2 topic function. Get Current should be negative when the robot is in the pool.
double BatteryMonitor::getCurrent() {

  return current;

}