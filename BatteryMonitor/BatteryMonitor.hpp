#ifndef BATTERYMONITOR_HPP
#define BATTERYMONITOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <iostream>
using namespace std::chrono_literals;

#define CURRENTRATING 10.5
#define MAXVOLT 16.8
#define MINVOLT 14

/// @TODO:
// Pack voltage
// Current sensor reading
//  Estimated remaining pack capacity based on total capacity and integrated
//  current reading

class BatteryMonitor : public rclcpp::Node {
public:
  BatteryMonitor()
      : Node("BatteryMonitorNode"),
        startupTime(std::chrono::steady_clock::now()) {
    // SetupADCConnection();
    CurrentChargeSub = this->create_subscription<std_msgs::msg::Float64>(
        "Current_Topic", 10,
        std::bind(&BatteryMonitor::getCurrent, this, std::placeholders::_1));
    CurrentVoltSub = this->create_subscription<std_msgs::msg::Float64>(
        "Volt_Topic", 10,
        std::bind(&BatteryMonitor::getVolt, this, std::placeholders::_1));
    timeInital = startupTime;
    // StartupSOC using Voltage at a single time point
    previousSOC = (CurrentVolt - MINVOLT / (MAXVOLT - MINVOLT)) / 100;

    // Callback CurrentCharge.
    SOCPublisher =
        this->create_publisher<std_msgs::msg::Float64>("SOCTopic", 10);
    SOCINTPublisher =
        this->create_publisher<std_msgs::msg::Bool>("SOCIntTopic", 10);

    // This function also generates INT for the SOCPublisherINT. If the call to
    // this function is removed, then make sure to replace with some way of
    // checking for INT of SOC.
    SOCtimer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                [this]() { this->SOC_RealTime_callback(); });
  }
  // Setup publisher to a SOC Topic.
private:
  // Implement c++ guidelines

  // test cases: StartupTime should be at power for Robot. Maybe try to look
  // through logs of when Pi started up?
  std::chrono::steady_clock::time_point startupTime;
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
  double CurrentVolt;
  void getCurrent(const std_msgs::msg::Float64::SharedPtr msg);
  double CurrentCurrent;
  void SOC_RealTime_callback();
};
#endif