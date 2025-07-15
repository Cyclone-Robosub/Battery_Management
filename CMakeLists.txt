cmake_minimum_required(VERSION 3.20)
project(batteryProject)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXPORT_COMPILE_COMMANDS ON)


add_executable( BatteryMontExecutable
    BatteryMonitor/BatteryMonitor.h
	BatteryMonitor/BatteryMonitor.cpp
)
ament_target_dependencies(BatteryMontExecutable rclcpp std_msgs)
install(TARGETS
  BatteryMontExecutable
  DESTINATION lib/${PROJECT_NAME})

  ament_package()