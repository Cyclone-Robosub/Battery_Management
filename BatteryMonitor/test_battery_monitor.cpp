#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include "BatteryMonitor.hpp"

//Made by AI from existing code.

class BatteryMonitorTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node = std::make_shared<BatteryMonitor>();

    // Setup test publishers
    volt_pub = node->create_publisher<std_msgs::msg::Float64>("Volt_Topic", 10);
    current_pub = node->create_publisher<std_msgs::msg::Float64>("Current_Topic", 10);

    // Setup test subscriptions
    soc_sub = node->create_subscription<std_msgs::msg::Float64>(
      "SOCTopic", 10,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        last_soc = msg->data;
        soc_received = true;
      });

    socint_sub = node->create_subscription<std_msgs::msg::Bool>(
      "SOCIntTopic", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        socint_triggered = msg->data;
        socint_received = true;
      });

    soc_received = false;
    socint_received = false;

    // Spin in a separate thread
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(node);
    spin_thread = std::thread([this]() {
      exec->spin();
    });
  }

  void TearDown() override {
    exec->cancel();
    spin_thread.join();
    rclcpp::shutdown();
  }

  std::shared_ptr<BatteryMonitor> node;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr volt_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_pub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr soc_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr socint_sub;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
  std::thread spin_thread;

  double last_soc = 0.0;
  bool soc_received = false;
  bool socint_triggered = false;
  bool socint_received = false;
};

TEST_F(BatteryMonitorTest, PublishesSOC) {
  std_msgs::msg::Float64 volt_msg;
  volt_msg.data = 15.5;  // Simulate voltage reading
  volt_pub->publish(volt_msg);

  std_msgs::msg::Float64 current_msg;
  current_msg.data = -1.0;  // Simulate current draw
  current_pub->publish(current_msg);

  rclcpp::sleep_for(std::chrono::milliseconds(200));

  // Allow time for callback and timer to trigger
  for (int i = 0; i < 10 && !soc_received; ++i) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(soc_received);
  EXPECT_GT(last_soc, -1.0);  // It should have calculated some SOC value
}

TEST_F(BatteryMonitorTest, TriggersSOCInterrupt) {
  std_msgs::msg::Float64 volt_msg;
  volt_msg.data = 14.1;
  volt_pub->publish(volt_msg);

  std_msgs::msg::Float64 current_msg;
  current_msg.data = -100.0;  // Simulate huge discharge
  current_pub->publish(current_msg);

  // Let SOC drop fast enough to hit DANGERSOCLEVEL
  rclcpp::sleep_for(std::chrono::seconds(1));

  for (int i = 0; i < 10 && !socint_received; ++i) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(socint_received);
  EXPECT_TRUE(socint_triggered);
}
