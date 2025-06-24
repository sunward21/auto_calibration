#!/bin/bash

# 获取脚本的绝对路径
#SCRIPT_PATH="$(realpath "$0")"
#SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
#BASE_DIR="$(dirname "$SCRIPT_DIR")"  # 获取auto-calibration目录
source /home/handsfree/GeoScan/GeoScan_SLAM/Fast_LVIO2_ws/devel/setup.bash
# 设置录制目录
BAG_DIR="/home/handsfree/GeoScan/GeoScan_Data/Camera_Bag"
#mkdir -p "$BAG_DIR"

# 启动相机驱动 - 使用绝对路径
#gnome-terminal -- bash -c "\"$BASE_DIR/camera_driver.sh\"; exec bash"
gnome-terminal -- bash -c "source ~/GeoScan/run_command/env.sh; roslaunch mvs_ros_pkg mvs_camera_trigger_indoor_rviz.launch; exec bash"
# 等待相机启动
echo "等待相机启动（5秒）..."
sleep 5

# 切换到录制目录
cd "$BAG_DIR" || exit 1

# 设置固定的文件名
BAG_NAME="xiangji.bag"
BAG_PATH="$BAG_DIR/$BAG_NAME"

# 检查文件是否已存在
if [ -f "$BAG_PATH" ]; then
    echo "错误: 文件 $BAG_NAME 已存在于 $BAG_DIR 中!"
    echo "为避免覆盖，请删除或重命名现有文件后再运行此脚本"
    exit 1
fi

# 开始录制
echo "开始录制相机数据包，将保存为: $BAG_NAME"
rosbag record /left_camera/image -O "$BAG_NAME"

echo "录制完成，数据包保存在: $BAG_PATH"
