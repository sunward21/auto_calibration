#!/bin/bash

# 显式加载ROS和您的工作空间环境

source /home/handsfree/GeoScan/GeoScan_SLAM/Fast_LVIO2_ws/devel/setup.bash

# 获取脚本的绝对路径
SCRIPT_PATH="$(realpath "$0")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
BASE_DIR="$(dirname "$SCRIPT_DIR")"  # 获取auto-calibration目录

# 启动雷达驱动
gnome-terminal -- bash -c "
    
    source /home/handsfree/GeoScan/GeoScan_SLAM/Fast_LVIO2_ws/devel/setup.bash;
    source ~/GeoScan/run_command/env.sh;
    roslaunch livox_ros_driver2 rviz_MID360.launch;
    exec bash"

sleep 5

# 启动相机驱动
gnome-terminal -- bash -c "
   
    source /home/handsfree/GeoScan/GeoScan_SLAM/Fast_LVIO2_ws/devel/setup.bash;
    source ~/GeoScan/run_command/env.sh;
    roslaunch mvs_ros_pkg mvs_camera_trigger2_indoor.launch;
    exec bash"

# 等待设备启动
echo "等待设备启动（10秒）..."
sleep 10

# 设置录制目录
BAG_DIR="/home/handsfree/GeoScan/GeoScan_Data/Calibration_Data"
mkdir -p $BAG_DIR
cd $BAG_DIR

# 录制3个不同角度的数据包，每次录制3秒
for i in {1..3}; do
    echo -e "\n准备录制第 ${i} 个数据包 (按回车开始录制，录制将持续3秒...)"
    read
    
    # 开始录制，3秒后自动停止
    echo "开始录制 (角度 ${i})..."
    rosbag record /left_camera/image /livox/imu /livox/lidar -O calibration_angle_${i}.bag --duration=3
    
    echo "第 ${i} 个数据包录制完成 (3秒)"
done

echo -e "\n所有数据包录制完成，保存在: $BAG_DIR"
echo "请确保每个角度都正确采集了数据"