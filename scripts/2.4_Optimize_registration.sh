#!/bin/bash

# 切换到工作空间目录
cd /home/handsfree/GeoScan/GeoScan_Calibration/direct_visual_lidar_calibration_ws

# 设置ROS工作空间环境
source devel/setup.bash

# 运行initial_guess_manual节点
rosrun direct_visual_lidar_calibration calibrate /home/handsfree/GeoScan/GeoScan_Data/Calibration_process