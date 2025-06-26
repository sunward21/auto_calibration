#!/bin/bash

# 设置工作目录
WORK_DIR="/home/handsfree/auto-calibration/scripts"

# 切换到工作目录
cd "$WORK_DIR" || {
    echo "错误：无法切换到目录 $WORK_DIR"
    exit 1
}

# 执行图片提取脚本
echo "正在标定鱼眼相机."
python3 1.4.2_calibration_fisher_camera.py

# 检查执行结果
if [ $? -eq 0 ]; then
    echo "标定完成！"
    echo "已经写入标定结果到/scripts/calib_result!!!!!"

else
    echo "错误：标定失败，请检查日志"
    exit 2
fi
