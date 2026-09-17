import os
import time
import math
import pybullet as p
import pybullet_data

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(PROJECT_DIR, "robot.urdf")

def get_angle(prompt):
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("Please enter a number, e.g. 30 or -20.")

print("\n========================================")
print("3-DOF ROBOT JOINT ANGLE TEST")
print("========================================")
print("Enter J1, J2 and J3 in degrees.\n")

j1_deg = get_angle("J1 (Base)    [deg]: ")
j2_deg = get_angle("J2 (Shoulder)[deg]: ")
j3_deg = get_angle("J3 (Wrist)   [deg]: ")

angles_deg = [j1_deg, j2_deg, j3_deg]
angles_rad = [math.radians(a) for a in angles_deg]

if not os.path.isfile(URDF_PATH):
    raise FileNotFoundError(f"Could not find robot.urdf here:\n{URDF_PATH}")

physics_client = p.connect(p.GUI)
if physics_client < 0:
    raise RuntimeError("Could not connect to the PyBullet GUI.")

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

print("\nLoading robot:")
print(URDF_PATH)

robot_id = p.loadURDF(
    URDF_PATH,
    basePosition=[0, 0, 0],
    useFixedBase=True
)

print("Robot loaded successfully. Robot ID:", robot_id)

expected = {
    "J1": "joint1_base_rotation",
    "J2": "joint2_shoulder",
    "J3": "joint3_wrist",
}

joint_indices = {}

for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    name = info[1].decode("utf-8")
    for label, expected_name in expected.items():
        if name == expected_name:
            joint_indices[label] = i

missing = [label for label in expected if label not in joint_indices]
if missing:
    raise RuntimeError(f"Could not find joints in robot.urdf: {missing}")

for label, angle_rad in zip(["J1", "J2", "J3"], angles_rad):
    p.resetJointState(robot_id, joint_indices[label], angle_rad)

print("\n========================================")
print("JOINT CONFIGURATION")
print("========================================")

for label, commanded_deg, commanded_rad in zip(
    ["J1", "J2", "J3"], angles_deg, angles_rad
):
    actual_rad = p.getJointState(robot_id, joint_indices[label])[0]
    print(
        f"{label}: command = {commanded_deg:7.2f} deg "
        f"({commanded_rad: .4f} rad) | "
        f"PyBullet = {math.degrees(actual_rad):7.2f} deg"
    )

p.resetDebugVisualizerCamera(
    cameraDistance=0.55,
    cameraYaw=45,
    cameraPitch=-25,
    cameraTargetPosition=[0, 0, 0.15]
)

print("\nPyBullet is running. Close the window to exit.\n")

try:
    while p.isConnected():
        p.stepSimulation()
        time.sleep(1.0 / 240.0)
except KeyboardInterrupt:
    print("\nSimulation stopped.")
finally:
    if p.isConnected():
        p.disconnect()
    print("PyBullet disconnected.")
