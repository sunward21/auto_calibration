#!/usr/bin/env python3
import re
import argparse
import sys
import os

def parse_matrix(matrix_str):
    """解析4x4矩阵字符串"""
    rows = matrix_str.strip().split('\n')
    matrix = []
    for row in rows:
        cleaned = row.strip()
        if not cleaned:
            continue
        nums = [float(x) for x in cleaned.split()[:4]]
        matrix.append(nums)
    return matrix

def format_rcl(r_values):
    """格式化Rcl为三行字符串"""
    return f"  Rcl: [{r_values[0]:.6f}, {r_values[1]:.6f}, {r_values[2]:.6f},\n" + \
           f"        {r_values[3]:.6f}, {r_values[4]:.6f}, {r_values[5]:.6f},\n" + \
           f"        {r_values[6]:.6f}, {r_values[7]:.6f}, {r_values[8]:.6f}]"

def format_pcl(p_values):
    """格式化Pcl为单行字符串"""
    return f"  Pcl: [{p_values[0]:.6f}, {p_values[1]:.6f}, {p_values[2]:.6f}]"

def find_extrin_calib_section(lines):
    """找到extrin_calib部分的开始和结束位置"""
    start_index = -1
    end_index = -1
    indent_level = None
    
    for i, line in enumerate(lines):
        # 找到extrin_calib标题
        if line.strip().startswith("extrin_calib:"):
            start_index = i
            # 确定缩进级别
            indent_level = len(line) - len(line.lstrip())
            continue
            
        if start_index != -1:
            # 如果遇到相同或更小缩进级别的行，表示结束
            current_indent = len(line) - len(line.lstrip())
            if line.strip() and current_indent <= indent_level and not line.strip().startswith('#'):
                end_index = i
                break
    
    # 如果没有明确结束，则到文件末尾
    if start_index != -1 and end_index == -1:
        end_index = len(lines)
    
    return start_index, end_index

def update_yaml_files(matrix, yaml_dir):
    """更新所有mid360相关的yaml文件，保留所有注释和格式"""
    if len(matrix) < 4 or any(len(row) < 4 for row in matrix[:4]):
        print(f"错误: 需要4x4矩阵, 但得到的是 {len(matrix)}x{len(matrix[0]) if matrix else 0} 矩阵")
        return
    
    # 提取旋转矩阵和平移向量
    R = [val for row in matrix[:3] for val in row[:3]]  # 3x3旋转矩阵展开为9元素列表
    P = [matrix[i][3] for i in range(3)]  # 前三行的第四列元素
    
    # 查找所有需要更新的yaml文件
    yaml_files = []
    for root, dirs, files in os.walk(yaml_dir):
        for file in files:
            if file.startswith("mid360") and file.endswith(".yaml"):
                yaml_files.append(os.path.join(root, file))
    
    if not yaml_files:
        print(f"在 {yaml_dir} 中未找到匹配的YAML文件")
        return
    
    for file_path in yaml_files:
        # 手动处理YAML文件，保留所有原始内容
        with open(file_path, 'r') as f:
            lines = f.readlines()
        
        # 准备新的Rcl和Pcl内容
        new_rcl_block = format_rcl(R) + "\n"
        new_pcl_line = format_pcl(P) + "\n"
        
        # 找到extrin_calib部分
        start_idx, end_idx = find_extrin_calib_section(lines)
        
        # 如果找不到extrin_calib部分，创建它
        if start_idx == -1:
            lines.append("\nextrin_calib:\n")
            lines.append(new_rcl_block)
            lines.append(new_pcl_line)
            with open(file_path, 'w') as f:
                f.writelines(lines)
            print(f"已更新文件: {file_path} (创建了新的extrin_calib部分)")
            continue
        
        # 处理extrin_calib部分
        extrin_section = lines[start_idx:end_idx]
        updated_extrin = []
        
        rcl_replaced = False
        pcl_replaced = False
        in_rcl_block = False
        in_pcl_block = False
        
        for line in extrin_section:
            # 跳过Rcl块内的所有内容
            if in_rcl_block:
                if ']' in line:
                    in_rcl_block = False
                continue
                
            # 跳过Pcl块内的所有内容
            if in_pcl_block:
                if ']' in line:
                    in_pcl_block = False
                continue
            
            # 处理Rcl部分 - 替换整个块
            if not rcl_replaced and line.strip().startswith('Rcl:'):
                # 添加新的Rcl块
                updated_extrin.append(new_rcl_block)
                rcl_replaced = True
                # 标记进入Rcl块，跳过所有后续内容直到结束
                if '[' in line and ']' not in line:
                    in_rcl_block = True
                continue
            
            # 处理Pcl部分 - 替换整个块
            if not pcl_replaced and line.strip().startswith('Pcl:'):
                # 添加新的Pcl行
                updated_extrin.append(new_pcl_line)
                pcl_replaced = True
                # 标记进入Pcl块，跳过所有后续内容直到结束
                if '[' in line and ']' not in line:
                    in_pcl_block = True
                continue
            
            # 保留原始行
            updated_extrin.append(line)
        
        # 如果没找到Rcl或Pcl，在extrin_calib部分添加
        if not rcl_replaced:
            # 在extrin_calib标题后添加
            if updated_extrin and updated_extrin[0].strip() == "extrin_calib:":
                updated_extrin.insert(1, new_rcl_block)
            else:
                updated_extrin.insert(0, new_rcl_block)
        
        if not pcl_replaced:
            # 在Rcl之后或extrin_calib标题后添加
            if rcl_replaced:
                # 找到Rcl位置
                rcl_idx = -1
                for i, line in enumerate(updated_extrin):
                    if 'Rcl:' in line:
                        rcl_idx = i
                        break
                if rcl_idx != -1:
                    updated_extrin.insert(rcl_idx + 1, new_pcl_line)
                else:
                    updated_extrin.append(new_pcl_line)
            else:
                if updated_extrin and updated_extrin[0].strip() == "extrin_calib:":
                    updated_extrin.insert(1, new_pcl_line)
                else:
                    updated_extrin.append(new_pcl_line)
        
        # 重组完整文件内容
        updated_lines = lines[:start_idx] + updated_extrin + lines[end_idx:]
        
        # 写入更新后的内容
        with open(file_path, 'w') as f:
            f.writelines(updated_lines)
        
        print(f"已更新文件: {file_path}")
    
    print("\n更新完成！")
    print("旋转矩阵 Rcl:")
    print(new_rcl_block.strip())
    print("平移向量 Pcl:")
    print(new_pcl_line.strip())

def main():
    parser = argparse.ArgumentParser(description='更新外参矩阵到YAML配置文件')
    parser.add_argument('matrix_file', help='包含4x4矩阵的文件路径')
    args = parser.parse_args()
    
    # 读取矩阵文件
    with open(args.matrix_file, 'r') as f:
        matrix_str = f.read()
    
    # 解析矩阵
    matrix = parse_matrix(matrix_str)
    
    if len(matrix) < 4 or any(len(row) < 4 for row in matrix):
        print(f"错误: 需要4x4矩阵, 但得到的是 {len(matrix)}x{len(matrix[0]) if matrix else 0} 矩阵")
        print("矩阵内容:")
        for row in matrix:
            print("  ", row)
        return
    
    print("解析的4x4矩阵:")
    for row in matrix:
        print("  ", row)
    
    # 更新所有相关yaml文件
    yaml_dir = "/home/handsfree/GeoScan/GeoScan_SLAM/Fast_LVIO2_ws/src/FAST-LIVO2/config"
    update_yaml_files(matrix, yaml_dir)

if __name__ == "__main__":
        main()
        # 添加等待用户输入，防止终端关闭
        input("\n程序执行完毕，按回车键或扫把退出...")