import numpy as np
import json

def quaternion_to_rotation_matrix(q):
    """将四元数 (qx, qy, qz, qw) 转换为3x3旋转矩阵"""
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,       2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,         1 - 2*qx**2 - 2*qz**2,   2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,         2*qy*qz + 2*qx*qw,       1 - 2*qx**2 - 2*qy**2]
    ])

def build_homogeneous_matrix(translation, quaternion):
    """构建4x4齐次变换矩阵"""
    R = quaternion_to_rotation_matrix(quaternion)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = translation
    return T

def inverse_homogeneous_matrix(T):
    """求齐次变换矩阵的逆"""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv

def format_matrix(T):
    """格式化矩阵为字符串输出"""
    return '\n'.join([' '.join(f"{x:.5f}" for x in row) for row in T])

# 主程序
if __name__ == "__main__":
    # 1. 读取JSON文件
    json_path = "/home/handsfree/GeoScan/GeoScan_Data/Calibration_process/calib.json"
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    # 2. 提取T_lidar_camera参数
    params = data["results"]["T_lidar_camera"]
    translation = params[:3]    # [tx, ty, tz]
    quaternion = params[3:]     # [qx, qy, qz, qw]
    
    # 3. 构建齐次变换矩阵并求逆
    T_lidar_camera = build_homogeneous_matrix(translation, quaternion)
    T_camera_lidar = inverse_homogeneous_matrix(T_lidar_camera)
    
    # 4. 写入输出文件
    output_path = "/home/handsfree/auto-calibration/scripts/extrinsic_matrix.txt"
    with open(output_path, 'w') as f:
        f.write(format_matrix(T_camera_lidar))
        f.write("\n")  # 添加换行符结尾
    
    print(f"转换完成！结果已保存至: {output_path}")
    input("\n程序执行完毕，按回车键或扫把退出...")