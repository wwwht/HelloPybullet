import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
boxId = p.loadURDF("../urdf/myfirst.urdf",startPos, startOrientation, useFixedBase=True)

# 设置Debug摄像头视角
# p.resetDebugVisualizerCamera( cameraDistance=2.7, cameraYaw=63,
# cameraPitch=-44, cameraTargetPosition=[0,0,0])
p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

# for i in range (1000):
while True:
    p.stepSimulation()
    time.sleep(1./240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

print(cubePos,cubeOrn)
p.disconnect()

