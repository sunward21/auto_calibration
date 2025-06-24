#!/bin/bash

# 设置工作目录
WORK_DIR="/home/handsfree/GeoScan/GeoScan_Calibration/camera_calibration"

# 切换到工作目录
cd "$WORK_DIR" || {
    echo "错误：无法切换到目录 $WORK_DIR"
    exit 1
}
source /home/handsfree/GeoScan/GeoScan_SLAM/Fast_LVIO2_ws/devel/setup.bash
# 执行图片提取脚本
echo "正在从数据包中提取图片..."
python3 extract_image.py

# 检查执行结果
if [ $? -eq 0 ]; then
    echo "图片提取完成！"
    echo "提取的图片保存在: /home/handsfree/GeoScan/GeoScan_Data/Image_Data"
    echo "请手动删除其中没有棋盘格的照片"
else
    echo "错误：图片提取失败，请检查日志"
    exit 2
fi