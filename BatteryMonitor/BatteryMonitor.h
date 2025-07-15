#include "BatteryMonitor.cpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
using namespace std::chrono_literals;

#define CURRENTRATING 10.5
#define MAXVOLT
#define MINVOLT

/// @TODO:
// Pack voltage
// Current sensor reading
//  Estimated remaining pack capacity based on total capacity and integrated
//  current reading

class BatteryMonitor : public rclcpp : Node {
public:
  BatteryMonitor() : Node("BatteryMonitorPublish") {
    // SetupADCConnection();
    timeInital = startupTime;
    // Need a startup SOC.
    previousSOC = (ReadVolt() - MINVOLT / (MAXVOLT - MINVOLT)) / 100;
    // Callback CurrentCharge.
    SOCPublisher =
        this->create_publisher<std_msgs::msg::Double>("SOCTopic", 10);

    auto SOC_RealTime_callback = [this]() -> void {
      this->SOCPublisher->publish(CalcSOC());
    };

    SOCtimer_ = this->create_wall_timer(100ms, SOC_RealTime_callback());
  }
  // Setup publisher to a SOC Topic.
private:
  // Implement c++ guidelines

  // test cases: StartupTime should be at power for Robot. Maybe try to look
  // through logs of when Pi started up?
  std::chrono::steady_clock::time_point startupTime =
      std::chrono::steady_clock::now();
  rclcpp::Publisher<std_msgs::msg::Double>::SharedPtr SOCPublisher;
  rclcpp::Subscriber<std_msgs::msg::String>::SharedPtr CurrentChargeSub;
  rclcpp::TimerBase::SharedPtr SOCtimer_;
  std::chrono::steady_clock::time_point timeInital;
  double currentCurrentValue;
  std::chrono::steady_clock::time_point timeFinal;
  const double currentRating = CURRENTRATING;
  // Need a startup SOC.
  double previousSOC;
  double CalcSOC();
  double ReadVolt();
};
