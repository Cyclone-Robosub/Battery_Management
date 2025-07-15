#include "BatteryMonitor.h"
#include <chrono>

/// @brief Step Function. 
double BatteryMonitor::CalcSOC() {
  double resultingSOC;
  // Get Time Final
  //  wait and set value for current value and Time Final.
  currentCurrentValue = getCurrent(); // get READING;
  timeFinal = std::chrono::steady_clock::now();
  auto deltaTime =
      std::chrono::duration<double>(timeFinal - timeInital).count();
  resultingSOC = previousSOC * (currentCurrentValue / currentRating) * (deltaTime);

  // Setup for next Reading of SOC
  previousSOC = resultingSOC;
  timeInital = timeFinal;

  return resultingSOC;
}
/// @brief Callback ROS2 topic function
double BatteryMonitor::ReadVolt(){
	double resultVolt;

	return resultVolt;
}
/// @brief Callback ROS2 topic function
double BatteryMonitor::getCurrent(){

}