#!/usr/bin/env python3
import re
import argparse
import sys
import os

def update_yaml(yaml_path, params):
    # 读取整个文件内容
    with open(yaml_path, 'r') as f:
        lines = f.readlines()
    
    # 更新需要的字段 - 逐行处理，只修改数值部分
    updated_lines = []
    for line in lines:
        # 更新cam_fx
        if line.strip().startswith('cam_fx:'):
            line = re.sub(r'(cam_fx:\s*)[\d.]+', r'\g<1>' + str(params['fx']), line)
        # 更新cam_fy
        elif line.strip().startswith('cam_fy:'):
            line = re.sub(r'(cam_fy:\s*)[\d.]+', r'\g<1>' + str(params['fy']), line)
        # 更新cam_cx
        elif line.strip().startswith('cam_cx:'):
            line = re.sub(r'(cam_cx:\s*)[\d.]+', r'\g<1>' + str(params['cx']), line)
        # 更新cam_cy
        elif line.strip().startswith('cam_cy:'):
            line = re.sub(r'(cam_cy:\s*)[\d.]+', r'\g<1>' + str(params['cy']), line)
        # 更新k1
        elif line.strip().startswith('k1:'):
            line = re.sub(r'(k1:\s*)[-\d.]+', r'\g<1>' + str(params['d'][0]), line)
        # 更新k2
        elif line.strip().startswith('k2:'):
            line = re.sub(r'(k2:\s*)[-\d.]+', r'\g<1>' + str(params['d'][1]), line)
        # 更新k3
        elif line.strip().startswith('k3:'):
            line = re.sub(r'(k3:\s*)[-\d.]+', r'\g<1>' + str(params['d'][2]), line)
        # 更新k4
        elif line.strip().startswith('k4:'):
            line = re.sub(r'(k4:\s*)[-\d.]+', r'\g<1>' + str(params['d'][3]), line)
        
        updated_lines.append(line)
    
    # 写入更新后的内容
    with open(yaml_path, 'w') as f:
        f.writelines(updated_lines)
    
    print(f"已更新配置文件: {yaml_path}")

def main():
    parser = argparse.ArgumentParser(description='处理相机内参标定结果')
    parser.add_argument('result_file', help='包含标定结果的文件路径')
    args = parser.parse_args()

    with open(args.result_file, 'r') as f:
        lines = f.readlines()
    
    params = {}
    
    # 逐行解析文件内容
    for line in lines:
        line = line.strip()
        
        # 解析DIM行
        if line.startswith("DIM="):
            try:
                # 提取括号内的尺寸
                dim_str = line.split('(', 1)[1].split(')', 1)[0]
                width, height = dim_str.split(',')
                params['width'] = int(width.strip())
                params['height'] = int(height.strip())
                print(f"提取到图像尺寸: {params['width']}x{params['height']}")
            except Exception as e:
                print(f"解析DIM行错误: {e}")
                continue
        
        # 解析K矩阵
        elif line.startswith("K=np.array"):
            try:
                # 提取数组内容
                k_str = line.split('[[', 1)[1].rsplit(']]', 1)[0]
                k_rows = k_str.split('], [')
                
                # 解析第一行
                row1 = k_rows[0].split(',')
                fx = float(row1[0].strip())
                cx = float(row1[2].strip())
                
                # 解析第二行
                row2 = k_rows[1].split(',')
                fy = float(row2[1].strip())
                cy = float(row2[2].strip())
                
                params['fx'] = fx
                params['fy'] = fy
                params['cx'] = cx
                params['cy'] = cy
                print(f"提取到内参矩阵: fx={fx}, fy={fy}, cx={cx}, cy={cy}")
            except Exception as e:
                print(f"解析K矩阵错误: {e}")
                continue
        
        # 解析D矩阵
        elif line.startswith("D=np.array"):
            try:
                # 提取数组内容
                d_str = line.split('[[', 1)[1].rsplit(']]', 1)[0]
                d_values = []
                
                # 解析所有数值
                for num_str in d_str.split('], ['):
                    # 移除可能的括号
                    num_str = num_str.replace('[', '').replace(']', '').strip()
                    if num_str:
                        d_values.append(float(num_str))
                
                if len(d_values) >= 4:
                    params['d'] = d_values[:4]
                    print(f"提取到畸变系数: k1={d_values[0]}, k2={d_values[1]}, k3={d_values[2]}, k4={d_values[3]}")
                else:
                    print(f"错误: 需要4个畸变系数，但只找到 {len(d_values)} 个")
            except Exception as e:
                print(f"解析D矩阵错误: {e}")
                continue
    
    # 检查是否提取到所有必要参数
    required_keys = ['width', 'height', 'fx', 'fy', 'cx', 'cy', 'd']
    missing = [key for key in required_keys if key not in params]
    
    if missing:
        print(f"错误: 缺少必要的参数: {', '.join(missing)}")
        return
    
    print("="*80)
    print("成功提取参数:")
    print(f"图像尺寸: {params['width']}x{params['height']}")
    print(f"焦距 (fx, fy): {params['fx']}, {params['fy']}")
    print(f"主点 (cx, cy): {params['cx']}, {params['cy']}")
    print(f"畸变系数 (k1, k2, k3, k4): {params['d']}")
    print("="*80)
    
    # 更新相机配置文件
    yaml_path = "/home/handsfree/GeoScan/GeoScan_SLAM/Fast_LVIO2_ws/src/FAST-LIVO2/config/camera_geoscan.yaml"
    update_yaml(yaml_path, params)
    
    # 生成preprocess命令
    preprocess_cmd = (
        f"rosrun direct_visual_lidar_calibration preprocess "
        f"/home/handsfree/GeoScan/GeoScan_Data/Calibration_Data -av "
        f"--camera_model=fisheye "
        f"--camera_intrinsics={params['fx']},{params['fy']},{params['cx']},{params['cy']} "
        f"--camera_distortion_coeffs={','.join(map(str, params['d']))} "
        f"/home/handsfree/GeoScan/GeoScan_Data/Calibration_process"
    )
    
    # 将命令输出到rosrun.txt文件（在当前文件夹）
    output_file = os.path.join(os.getcwd(), "rosrun.txt")
    with open(output_file, 'w') as f:
        f.write(preprocess_cmd)
    
    print(f"已将rosrun命令写入: {output_file}")
    print("="*80)
    print("请使用以下命令进行后续处理:")
    print(preprocess_cmd)
    print("="*80)

if __name__ == "__main__":
    main()