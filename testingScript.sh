 #!/bin/bash

 source /opt/ros/jazzy/setup.bash
 colcon build --cmake-args -DBUILD_TESTING=ON
 cd build/battery_management
 ./battery_management_test > ../../Latest_gtest_output.txt
 