#!/bin/bash

# 切换到工作空间目录
cd /home/handsfree/GeoScan/GeoScan_Calibration/direct_visual_lidar_calibration_ws

# 设置工作空间环境
source devel/setup.bash

# 获取并执行rosrun.txt中的命令
if [ -f /home/handsfree/auto-calibration/scripts/rosrun.txt ]; then
    echo "正在执行rosrun.txt中的命令:"
    cat /home/handsfree/auto-calibration/scripts/rosrun.txt
    echo ""
    bash /home/handsfree/auto-calibration/scripts/rosrun.txt
else
    echo "错误: 未找到rosrun.txt文件"
    echo "请确保在运行此脚本前已生成rosrun.txt"
fi
