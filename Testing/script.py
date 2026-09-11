import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

arm_id = p.loadURDF("3_dof.urdf", basePosition=[0,0,0], useFixedBase=True)

# List joint indices/names -- PyBullet numbers them in URDF-definition order
for i in range(p.getNumJoints(arm_id)):
    info = p.getJointInfo(arm_id, i)
    print(i, info[1].decode())

# Command your IK-computed angles directly
theta1, theta2, theta3 = 0.5, 0.8, 0.3   # radians, from your solve_ik()
p.setJointMotorControl2(arm_id, 0, p.POSITION_CONTROL, targetPosition=theta1)
p.setJointMotorControl2(arm_id, 1, p.POSITION_CONTROL, targetPosition=theta2)
p.setJointMotorControl2(arm_id, 2, p.POSITION_CONTROL, targetPosition=theta3)

for _ in range(1000):
    p.stepSimulation()
    time.sleep(1/240)