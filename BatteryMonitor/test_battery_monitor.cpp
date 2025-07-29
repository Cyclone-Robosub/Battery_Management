#include "BatteryMonitor.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
//Initial design by AI

class BatteryMonitorTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node = std::make_shared<BatteryMonitor>();
    std::thread StartupThread(&BatteryMonitor::Startup, &(*node));
    StartupThread.detach();
    // Setup test publishers
    volt_pub = node->create_publisher<std_msgs::msg::Float64>("Volt_Topic", 10);
    current_pub =
        node->create_publisher<std_msgs::msg::Float64>("Current_Topic", 10);

    // Setup test subscriptions
    soc_sub = node->create_subscription<std_msgs::msg::Float64>(
        "SOCTopic", 10, [this](std_msgs::msg::Float64::SharedPtr msg) {
          last_soc = msg->data;
          soc_received = true;
        });

    socint_sub = node->create_subscription<std_msgs::msg::Bool>(
        "SOCIntTopic", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
          socint_triggered = msg->data;
          socint_received = true;
        });

    soc_received = false;
    socint_received = false;

    // Spin in a separate thread
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(node);
    spin_thread = std::thread([this]() { exec->spin(); });
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
  volt_msg.data = 16.7; // Simulate voltage reading
  volt_pub->publish(volt_msg);

  std_msgs::msg::Float64 current_msg;
  current_msg.data = -15.0; // Simulate current draw
  current_pub->publish(current_msg);

  rclcpp::sleep_for(std::chrono::milliseconds(200));
  for (int i = 0; i < 10; ++i) {
    current_pub->publish(current_msg);
    current_msg.data -= 20;
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  // Allow time for callback and timer to trigger
  for (int i = 0; i < 10 && !soc_received; ++i) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_TRUE(soc_received);
  EXPECT_GT(last_soc, -1.0); // It should have calculated some SOC value
}

TEST_F(BatteryMonitorTest, TriggersSOCInterrupt) {
  std_msgs::msg::Float64 volt_msg;
  volt_msg.data = 14.005;
  volt_pub->publish(volt_msg);

  std_msgs::msg::Float64 current_msg;
  current_msg.data = -10000.0; // Simulate huge discharge
  current_pub->publish(current_msg);

  // Let SOC drop fast enough to hit DANGERSOCLEVEL
  rclcpp::sleep_for(std::chrono::seconds(1));

  for (int i = 0; i < 10 && !socint_received; ++i) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(socint_received);
  EXPECT_TRUE(socint_triggered);
}

TEST_F(BatteryMonitorTest, ContinuousSOCPublishing) {
  // Simulate 10 iterations of input to simulate runtime
  std_msgs::msg::Float64 volt_msg;
  std_msgs::msg::Float64 current_msg;

  volt_msg.data = 15.0;     // Mid-level voltage
  current_msg.data = -14.0; // Mild current draw

  int messages_received = 0;

  // Reset subscription to count multiple SOC messages
  soc_sub = node->create_subscription<std_msgs::msg::Float64>(
      "SOCTopic", 10,
      [&messages_received](std_msgs::msg::Float64::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("TestLogger"), "SOC: %.2f", msg->data);
        ++messages_received;
      });

  for (int i = 0; i < 10; ++i) {
    current_pub->publish(current_msg);
    current_msg.data -= 20;
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_GE(messages_received, 5)
      << "Expected at least 5 SOC messages over time";
}
