#include "BatteryMonitor.h"
/// @brief Step Function. 
double BatteryMonitor::ReadSOC() {
  double resultingSOC;
  // Get Time Final
  //  wait and set value for current value and Time Final.
  currentCurrentValue; // get READING;
  timeFinal = std::chrono::steady_clock::now();
  auto deltaTime =
      std::chrono::duration<double>(timeFinal - timeInital).count();
  resultingSOC = previousSOC * (currentCurrentValue / currentRating) * (deltaTime);

  // Setup for next Reading of SOC
  previousSOC = resultingSOC;
  timeInital = timeFinal;

  return resultingSOC;
}