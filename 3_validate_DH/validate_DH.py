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
    
    # T_end_effector = np.array([
    # [1, 0, 0, 0],
    # [0, 1, 0, 0.1],
    # [0, 0, 1, 0],
    # [0, 0, 0, 1]
    # ])
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
    [0, 0, 1, np.pi / 2],     # 示例：第一关节
    [0.1, np.pi / 2, 0.1, np.pi / 4], # 示例：第二关节
    [0, np.pi / 2, 0, np.pi / 4]  # 示例：第三关节
]

mdh_transforms = calculate_forward_kinematics(mdh_params)
target_pos = mdh_transforms[-1]  # 提取末端位置
rotation = target_pos[:3, :3]
position = target_pos[:3, 3]


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

# mdh_positions = [transform[:3, 3] for transform in mdh_transforms]  # 提取位置

# physicsClient = p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0,0,-10)

# planeId = p.loadURDF("plane.urdf")
# startPos = [0,0,0]
# startOrientation = p.getQuaternionFromEuler([0,0,0])
# # boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
# boxId = p.loadURDF("../urdf/new_robot.urdf",startPos, startOrientation, useFixedBase=True)

# num_joints = p.getNumJoints(boxId)
# urdf_joint_positions = []
# for joint_index in range(num_joints):
#     link_state = p.getLinkState(boxId, joint_index)
#     urdf_joint_positions.append(link_state[4])  # link_state[4] 是世界坐标系中的位置

# for i, (mdh_pos, urdf_pos) in enumerate(zip(mdh_positions, urdf_joint_positions)):
#     print(f"Joint {i}:")
#     print(f"  MDH Position: {mdh_pos}")
#     print(f"  URDF Position: {urdf_pos}")
#     error = np.linalg.norm(np.array(mdh_pos) - np.array(urdf_pos))  # 计算欧几里得距离
#     print(f"  Position Error: {error}")

# # 设置Debug摄像头视角
# # p.resetDebugVisualizerCamera( cameraDistance=2.7, cameraYaw=63,
# # cameraPitch=-44, cameraTargetPosition=[0,0,0])
# p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

# prismatic_joint_index = 0  # prismatic_joint
# wrist_1_joint_index = 1    # wrist_1_joint
# wrist_2_joint_index = 2    # wrist_2_joint
# end_effector_index = 3
# num_joints = p.getNumJoints(boxId)
# for joint_index in range(num_joints):
#     link_state = p.getLinkState(boxId, joint_index)
#     print(link_state[4])


# prismatic_slider = p.addUserDebugParameter("Prismatic Joint", 0, 1, 0.5)  # 滑块初始值 0.5，范围 [0, 1]
# wrist_1_slider = p.addUserDebugParameter("Wrist 1 Joint", -3.14, 3.14, 0)  # 滑块初始值 0，范围 [-π, π]
# wrist_2_slider = p.addUserDebugParameter("Wrist 2 Joint", -3.14, 3.14, 0)  # 滑块初始值 0，范围 [-π, π]
    
# while True:
#     p.stepSimulation()

#     prismatic_target = p.readUserDebugParameter(prismatic_slider)
#     wrist_1_target = p.readUserDebugParameter(wrist_1_slider)
#     wrist_2_target = p.readUserDebugParameter(wrist_2_slider)

#     # 使用调试滑块值控制关节
#     p.setJointMotorControl2(
#         bodyIndex=boxId,
#         jointIndex=prismatic_joint_index,
#         controlMode=p.POSITION_CONTROL,
#         targetPosition=prismatic_target,  # 滑块的目标位置
#         force=50
#     )

#     p.setJointMotorControl2(
#         bodyIndex=boxId,
#         jointIndex=wrist_1_joint_index,
#         controlMode=p.POSITION_CONTROL,
#         targetPosition=wrist_1_target,  # 滑块的目标角度
#         force=50
#     )

#     p.setJointMotorControl2(
#         bodyIndex=boxId,
#         jointIndex=wrist_2_joint_index,
#         controlMode=p.POSITION_CONTROL,
#         targetPosition=wrist_2_target,  # 滑块的目标角度
#         force=50
#     )


#     # 获取末端执行器的位置信息
#     link_state = p.getLinkState(boxId, end_effector_index)
#     position = link_state[0]  # 获取位置 (x, y, z)
#     orientation = link_state[1]  # 获取姿态 (四元数)

#     position = [round(coord, 2) for coord in position]
#     orientation = [round(quat, 2) for quat in orientation]

#     # # 打印实时位置
#     # print(f"End Effector (camera_link) Position: {position}")
#     # print(f"End Effector (camera_link) Orientation (quaternion): {orientation}")

#     # 在仿真场景中显示位置信息
#     p.addUserDebugText(
#         text=f"Pos: {position}\n Ori: {orientation}",
#         textPosition=[0,0,0.5],
#         textColorRGB=[1, 0, 0],
#         textSize=1.2,
#         lifeTime=1  # 文字在场景中存活时间
#     )

#     # 设置步长时间
#     time.sleep(1. / 240.)