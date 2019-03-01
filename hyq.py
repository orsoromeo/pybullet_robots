import pybullet as p
import numpy as np
import math
from ikpy.chain import Chain
from datetime import datetime
import time

p.connect(p.GUI)
plane = p.loadURDF("plane.urdf", [0, 0, -0.5])
obstacle1 = p.loadURDF("boston_box.urdf", [0, 2, 0])
obstacle2 = p.loadURDF("marble_cube.urdf", [0, 3, 0])
obstacle3 = p.loadURDF("boston_box.urdf", [0, 4, 0])
obstacle4 = p.loadURDF("marble_cube.urdf", [0, 5, 0])
obstacle5 = p.loadURDF("marble_cube.urdf", [0, 4, 1])
obstacle6 = p.loadURDF("r2d2.urdf", [0, 2, 1])
p.setGravity(0, 0, -9.8)
simulationFreq = 1. / 500
p.setTimeStep(simulationFreq)
# p.setDefaultContactERP(0)
# urdfFlags = p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
urdfFlags = p.URDF_USE_SELF_COLLISION
quadruped = p.loadURDF("hyq/hyq.urdf", [0, 0, 0], [0, 0.5, 0.5, 0], flags=urdfFlags, useFixedBase=False)

my_chain = Chain.from_urdf_file("data/hyq/hyq.urdf")

# enable collision between lower legs

for j in range(p.getNumJoints(quadruped)):
    print(p.getJointInfo(quadruped, j))

# 2,5,8 and 11 are the lower legs
lower_legs = [2, 5, 8, 11]
for l0 in lower_legs:
    for l1 in lower_legs:
        if (l1 > l0):
            enableCollision = 1
            print("collision for pair", l0, l1, p.getJointInfo(quadruped, l0)[12], p.getJointInfo(quadruped, l1)[12],
                  "enabled=", enableCollision)
            p.setCollisionFilterPair(quadruped, quadruped, 2, 5, enableCollision)

jointIds = []
paramIds = []
jointOffsets = []
jointDirections = [-1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
jointAngles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

for i in range(4):
    jointOffsets.append(0)
    jointOffsets.append(-0.7)
    jointOffsets.append(0.7)
    jointOffsets.append(0.0)

maxForceId = p.addUserDebugParameter("maxForce", 0, 100, 20)
numJoints = p.getNumJoints(quadruped)

for j in range(p.getNumJoints(quadruped)):
    p.changeDynamics(quadruped, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(quadruped, j)
    # print(info)
    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
        jointIds.append(j)

p.getCameraImage(480, 320)
p.setRealTimeSimulation(0)

joints = []

#with open("data1.txt", "r") as filestream:
#    for line in filestream:
#        maxForce = p.readUserDebugParameter(maxForceId)
#        currentline = line.split(",")
#        frame = currentline[0]
#        t = currentline[1]
#        joints = currentline[2:14]
#        for j in range(12):
#            targetPos = float(joints[j])
#            p.setJointMotorControl2(quadruped, jointIds[j], p.POSITION_CONTROL,
#                                    jointDirections[j] * targetPos + jointOffsets[j], force=maxForce)
#        p.stepSimulation()
#        for lower_leg in lower_legs:
#            # print("points for ", quadruped, " link: ", lower_leg)
#            pts = p.getContactPoints(quadruped, -1, lower_leg)
#        # print("num points=",len(pts))
#        # for pt in pts:
#        #	print(pt[9])
#        time.sleep(1. / 500.)

for j in range(p.getNumJoints(quadruped)):
    p.changeDynamics(quadruped, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(quadruped, j)
    js = p.getJointState(quadruped, j)
    # print(info)
    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
        paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, (js[0] - jointOffsets[j]) / jointDirections[j]))

p.setRealTimeSimulation(1)

sawyerEndEffectorIndex = [3, 7, 11, 15]
cyrcle_center = [0.2, -0.2, 0.2, -0.2, 0.3, 0.3, -0.3, -0.3]
numJoints = p.getNumJoints(quadruped)
jd=[0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001]
useRealTimeSimulation = 0
t = 0
prevPose=[0,0,0]
prevPose1=[0,0,0]
hasPrevPose = 0
trailDuration = 15
costant = [0, 1.8, 1.8, 0]
old_base_pos, old_base_orient = p.getBasePositionAndOrientation(quadruped)
lin_vel=[0,0,0]
ang_vel=[0,0,0]
body_xyz, orient = p.getBasePositionAndOrientation(quadruped)

while (1):

    for i in range(len(paramIds)):
        c = paramIds[i]
        targetPos = p.readUserDebugParameter(c)
        maxForce = p.readUserDebugParameter(maxForceId)
        p.setJointMotorControl2(quadruped, jointIds[i], p.POSITION_CONTROL,
                                jointDirections[i] * targetPos + jointOffsets[i], force=maxForce)

    FR = p.getJointInfo(quadruped, 3)
    FR_xyz = FR[14]
    FL = p.getJointInfo(quadruped, 7)
    FL_xyz = FL[14]
    RR = p.getJointInfo(quadruped, 11)
    RR_xyz = RR[14]
    RL = p.getJointInfo(quadruped, 15)
    RL_xyz = RL[14]
    body_x = (FR_xyz[0] + FL_xyz[0] + RR_xyz[0] + RL_xyz[0])/4 - body_xyz[0]
    body_y = (FR_xyz[1] + FL_xyz[1] + RR_xyz[1] + RL_xyz[1])/4 - body_xyz[1]
    if body_x != 0:
        body_xyz[0] + (math.fabs(body_x) / body_x) * 0.1
    if body_y != 0:
        body_xyz[1] + (math.fabs(body_y) / body_y) * 0.1
    p.resetBasePositionAndOrientation(quadruped, [old_base_pos[0], old_base_pos[1] + 5*simulationFreq, 0], orient)

    if (useRealTimeSimulation):
        dt = datetime.now()
        t = (dt.second / 60.) * 2. * math.pi
    else:
        t += 0.01
        time.sleep(0.01)

    for i in range(4):
        pos = [body_xyz[0] + cyrcle_center[i], body_xyz[1] + cyrcle_center[i+4] + 0.15 * math.sin(8.5*(t + costant[i])), 0.1 * math.cos(8.5*(t + costant[i])) + body_xyz[2] - 0.4]
        jointPoses = p.calculateInverseKinematics(quadruped, sawyerEndEffectorIndex[i], pos, jointDamping=jd)

        # reset the joint state (ignoring all dynamics, not recommended to use during simulation)
        for i in range(numJoints):
            jointInfo = p.getJointInfo(quadruped, i)
            qIndex = jointInfo[3]
            if qIndex > -1:
                p.resetJointState(quadruped, i, jointPoses[qIndex - 7])


    body_xyz, orient = p.getBasePositionAndOrientation(quadruped)
    lin_vel = np.subtract(body_xyz, old_base_pos)*simulationFreq
    old_base_pos = body_xyz
    old_base_orient = orient
