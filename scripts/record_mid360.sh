#!/bin/zsh

# Record mid360 rosbag
source install/setup.zsh

ros2 bag record -o real_car_$(date +%Y%m%d_%H%M%S) \
  livox/lidar \
  livox/imu \
  joint_states
