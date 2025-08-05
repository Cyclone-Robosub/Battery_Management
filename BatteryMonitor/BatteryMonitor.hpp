#ifndef BATTERYMONITOR_HPP
#define BATTERYMONITOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <iostream>
using namespace std::chrono_literals;

#define CURRENTRATING 36000
#define MAXVOLT 16.8
#define MINVOLT 14

/// @TODO:
// Pack voltage
// Current sensor reading
//  Estimated remaining pack capacity based on total capacity and integrated
//  current reading

class BatteryMonitor : public rclcpp::Node {
public:
  BatteryMonitor() : Node("BatteryMonitorNode") {
    SetupADCConnection();
    StartupROS();
   // std::thread StartupThread(&BatteryMonitor::Startup, this);
   // StartupThread.detach();
  }
  // Setup publisher to a SOC Topic.
  void Startup();
  void Shutdown();

private:
  // Implement c++ guidelines
  void SetupADCConnection();
  void StartupROS();
  // test cases: StartupTime should be at power for Robot. Maybe try to look
  // through logs of when Pi started up?
  rclcpp::CallbackGroup::SharedPtr callbackBattery;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr SOCPublisher;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr CurrentChargeSub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr CurrentVoltSub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr SOCINTPublisher;
  rclcpp::TimerBase::SharedPtr SOCtimer_;

  std::chrono::steady_clock::time_point timeInital;
  double currentCurrentValue;
  std::chrono::steady_clock::time_point timeFinal;
  const double currentRating = CURRENTRATING;
  double previousSOC;
  double CalcSOC();

  void getVolt(const std_msgs::msg::Float64::SharedPtr msg);
  std::optional<double> CurrentVolt;
  void getCurrent(const std_msgs::msg::Float64::SharedPtr msg);
  std::optional<double> CurrentCurrent;
  void SOC_RealTime_callback();
  std::atomic<bool> voltReceived{false};
  std::atomic<bool> KeepGoing{true};
};
#endif