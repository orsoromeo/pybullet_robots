import pybullet as p
import math
from ikpy.chain import Chain
from datetime import datetime
import time

p.connect(p.GUI)
plane = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)
p.setTimeStep(1. / 500)
# p.setDefaultContactERP(0)
# urdfFlags = p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
urdfFlags = p.URDF_USE_SELF_COLLISION
quadruped = p.loadURDF("hyq/hyq.urdf", [0, 0, 0.5], [0, 0.5, 0.5, 0], flags=urdfFlags, useFixedBase=False)

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

sawyerEndEffectorIndex = 3
numJoints = p.getNumJoints(quadruped)
jd=[0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001]
useRealTimeSimulation = 0
t = 0
prevPose=[0,0,0]
prevPose1=[0,0,0]
hasPrevPose = 0
trailDuration = 15

while (1):

    for i in range(len(paramIds)):
        c = paramIds[i]
        targetPos = p.readUserDebugParameter(c)
        maxForce = p.readUserDebugParameter(maxForceId)
        p.setJointMotorControl2(quadruped, jointIds[i], p.POSITION_CONTROL,
                                jointDirections[i] * targetPos + jointOffsets[i], force=maxForce)



    if (useRealTimeSimulation):
        dt = datetime.now()
        t = (dt.second / 60.) * 2. * math.pi
    else:
        t = t + 0.01
        time.sleep(0.01)

    for i in range(1):
        pos = [0.2 + 0.1 * math.cos(t), 0.2 + 0.1 * math.sin(t), 0.05]
        jointPoses = p.calculateInverseKinematics(quadruped, sawyerEndEffectorIndex, pos, jointDamping=jd)

        # reset the joint state (ignoring all dynamics, not recommended to use during simulation)
        for i in range(numJoints):
            jointInfo = p.getJointInfo(quadruped, i)
            qIndex = jointInfo[3]
            if qIndex > -1:
                p.resetJointState(quadruped, i, jointPoses[qIndex - 7])

    ls = p.getLinkState(quadruped, sawyerEndEffectorIndex)
    if (hasPrevPose):
        p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
        p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = 1

