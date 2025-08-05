cmake_minimum_required(VERSION 3.20)
project(battery_management)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXPORT_COMPILE_COMMANDS ON)


add_executable(BatteryMontExecutable
  BatteryMonitor/main.cpp
  BatteryMonitor/BatteryMonitor.cpp
)
ament_target_dependencies(BatteryMontExecutable rclcpp std_msgs)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(battery_management_test
    BatteryMonitor/test_battery_monitor.cpp
     BatteryMonitor/BatteryMonitor.cpp
     BatteryMonitor/BatteryMonitor.hpp
  )
  ament_target_dependencies(battery_management_test
    rclcpp
    std_msgs
  )



install(TARGETS
  BatteryMontExecutable
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()