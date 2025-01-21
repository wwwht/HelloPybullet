import pybullet as p
import pybullet_data
import time
import numpy as np
import tempfile
import os

def mdh_to_transformation(a, alpha, d, theta):
    """根据MDH参数生成变换矩阵"""
    transform = np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
        [0, 0, 0, 1]
    ])
    return transform

def calculate_forward_kinematics(mdh_params):
    """根据MDH参数计算从基座到每个关节的变换矩阵"""
    transforms = []
    current_transform = np.eye(4)  # 初始为单位矩阵（基座位置）
    for a, alpha, d, theta in mdh_params:
        t = mdh_to_transformation(a, alpha, d, theta)
        current_transform = np.dot(current_transform, t)
        transforms.append(current_transform)
    
    T_end_effector = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0.1],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
    ])
    # current_transform = np.dot(current_transform, T_end_effector)
    # transforms.append(current_transform)
    return transforms

def calculate_inverse_kinematics(target_pos, T_end_effector):
    # 提取目标位置
    # T_target_base_to_joint_n = np.dot(target_pos, np.linalg.inv(T_end_effector))

    # 提取目标位置
    x = target_pos[0, 3]
    y = target_pos[1, 3]
    z = target_pos[2, 3]

    # 提取目标旋转矩阵
    R = target_pos[:3, :3]
    
    # 1. 求解 p2
    try:
        p2 = np.arctan2(R[1,2] ,  -1 * R[2,2]) # 根据公式: p2 = arccos(x / 0.1)
    except ValueError:
        raise ValueError("目标位置超出机械臂工作空间，x 值超出范围！")

    # 检查解的多值性
    if np.isnan(p2):
        raise ValueError("无法求解 p2：计算结果无效。")

    # 2. 求解 p1
    p1 = z  # 根据公式: p1 = z + 0.1 * sin(p2)

    # 3. 求解 p3
    # 使用旋转矩阵的 R[1, 0] 和 R[1, 1]
   
    p3 = np.arctan2(R[0,0], R[0,1])  # 根据公式: p3 = arctan2(R[1, 0], R[1, 1])

    return p1, p2, p3
    

base_position = [0, 0, 0]  # 基座位置
base_orientation = [0, 0, 0, 1]  # 无旋转

# MDH : a, alpha, d, theta
# mdh_params = [
#     [0, 0, 0.1, 0],     # 示例：第一关节
#     [0, -np.pi / 2, 0.1, -np.pi / 4], # 示例：第二关节
#     [0, -np.pi / 2, 0, 0],  # 示例：第三关节
#     [0, 0, 0.1, 0]
# ]

mdh_params = [
    [0, 0, 0.3, np.pi / 2],     # 示例：第一关节
    [0.1, np.pi / 2, 0.1, np.pi / 4], # 示例：第二关节
    [0, np.pi / 2, 0, np.pi / 4]  # 示例：第三关节
]

mdh_transforms = calculate_forward_kinematics(mdh_params)
target_pos = mdh_transforms[-1]  # 提取末端位置
print(f"末端位置: {target_pos}")

T_end_effector = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0.1],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
    ])

import math
p1, p2, p3 = calculate_inverse_kinematics(target_pos, T_end_effector)
p2 = math.degrees(p2)
p3 = math.degrees(p3)
print(f"逆解: p1={p1}, p2={p2}, p3={p3}")