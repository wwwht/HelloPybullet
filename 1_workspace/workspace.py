import pybullet as p
import time
import numpy as np
import pybullet_data

# 关节采样函数
def sample_joint_positions(joint_limits, num_samples=100):
    joint_positions = []
    for i in range(len(joint_limits)):
        low, high = joint_limits[i]
        if low < high:  # 如果有范围限制
            joint_positions.append(np.random.uniform(low, high, num_samples))
        else:
            joint_positions.append([0] * num_samples)  # 固定关节
            print(f"Joint {i} is fixed.")
    return np.array(joint_positions).T

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
robot_id  = p.loadURDF("../urdf/myfirst.urdf",startPos, startOrientation, useFixedBase=True)

# 设置Debug摄像头视角
# p.resetDebugVisualizerCamera( cameraDistance=2.7, cameraYaw=63,
# cameraPitch=-44, cameraTargetPosition=[0,0,0])
p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

# 获取机械臂关节信息
num_joints = p.getNumJoints(robot_id)
joint_limits = []
for joint_index in range(num_joints):
    info = p.getJointInfo(robot_id, joint_index)
    joint_limits.append((info[8], info[9]))  # lower and upper limits
    print(f"Joint {joint_index}: {info[1].decode('utf-8')} - Limits: {info[8]}, {info[9]}")

def visualize_workspace(robot_id, joint_limits, p_client, num_samples=1000):
    points = []
    joint_samples = sample_joint_positions(joint_limits, num_samples)


    for joint_positions in joint_samples:
        for i in range(len(joint_positions)):
            p.resetJointState(robot_id, i, joint_positions[i])

        # 获取末端执行器位置
        link_state = p.getLinkState(robot_id, num_joints - 1)  # 获取最后一个 link 的状态
        end_effector_position = link_state[4]  # 世界坐标中的末端执行器位置
        points.append(end_effector_position)

    # 在 PyBullet 中绘制可达空间
    print("可达空间点数:", len(points))
    for point in points:
        offset = 0.01  # 偏移量
        start_point = point
        end_point = [point[0] + offset, point[1], point[2]]  # 在 x 方向上偏移一点
        p_client.addUserDebugLine(start_point, end_point, [0, 1, 0], 1)  # 绿色小线段

visualize_workspace(robot_id, joint_limits, p, num_samples=1000)

# for i in range (1000):
while True:
    p.stepSimulation()
    time.sleep(1./240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

print(cubePos,cubeOrn)
p.disconnect()

