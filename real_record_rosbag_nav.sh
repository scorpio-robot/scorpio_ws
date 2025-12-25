#!/bin/zsh

# Record nav rosbag in real world
source install/setup.zsh

ros2 bag record -o real_car_mid360_$(date +%Y%m%d_%H%M%S) \
  /joint_states \
  /livox/imu \
  /livox/lidar