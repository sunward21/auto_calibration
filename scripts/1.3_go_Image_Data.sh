#!/bin/bash

# 设置目标目录
TARGET_DIR="/home/handsfree/GeoScan/GeoScan_Data/Image_Data"

# 检查nautilus是否可用
if ! command -v nautilus &> /dev/null; then
    echo "错误：nautilus 文件管理器未安装"
    echo "尝试安装？(y/n)"
    read install_nautilus
    if [ "$install_nautilus" = "y" ] || [ "$install_nautilus" = "Y" ]; then
        sudo apt-get update
        sudo apt-get install -y nautilus
    else
        echo "请手动安装 nautilus 或使用其他文件管理器"
        exit 1
    fi
fi

# 检查目录是否存在
if [ ! -d "$TARGET_DIR" ]; then
    echo "目录 $TARGET_DIR 不存在"
    echo "是否创建该目录并打开？(y/n)"
    read create_dir
    if [ "$create_dir" = "y" ] || [ "$create_dir" = "Y" ]; then
        mkdir -p "$TARGET_DIR"
        echo "目录已创建: $TARGET_DIR"
    else
        echo "操作取消"
        exit 1
    fi
fi

# 获取目录中的文件数量
file_count=$(find "$TARGET_DIR" -maxdepth 1 -type f | wc -l)

# 打开文件管理器
echo "正在打开图片目录 ($file_count 个文件)..."
nautilus "$TARGET_DIR" > /dev/null 2>&1 &

# 提示用户操作
echo "文件管理器已打开，显示目录: $TARGET_DIR"
echo "该目录包含 $file_count 个文件"
echo "请手动删除其中没有棋盘格的照片"

echo "删完之后可用扫把杀掉终端"
 
exit 0