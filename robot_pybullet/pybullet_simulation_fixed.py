import os
import time

import pybullet as p
import pybullet_data


# ============================================================
# PYBULLET ROBOT VIEWER
# ============================================================

# Find the folder containing this Python file
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))

# Our URDF is in the same folder
URDF_PATH = os.path.join(PROJECT_DIR, "robot.urdf")


# ============================================================
# CHECK URDF
# ============================================================

if not os.path.isfile(URDF_PATH):
    raise FileNotFoundError(
        f"\nCould not find robot.urdf here:\n{URDF_PATH}\n"
    )


# ============================================================
# CONNECT TO PYBULLET
# ============================================================

physics_client = p.connect(p.GUI)

if physics_client < 0:
    raise RuntimeError("Could not connect to PyBullet.")


# ============================================================
# PYBULLET SETTINGS
# ============================================================

p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)

p.setGravity(0, 0, -9.81)


# ============================================================
# LOAD GROUND
# ============================================================

p.loadURDF("plane.urdf")


# ============================================================
# LOAD OUR ROBOT
# ============================================================

print("\n========================================")
print("Loading robot")
print("========================================")

print(URDF_PATH)

robot_id = p.loadURDF(
    URDF_PATH,
    basePosition=[0, 0, 0],
    useFixedBase=True
)

print("\nRobot loaded successfully.")
print("Robot ID:", robot_id)


# ============================================================
# PRINT JOINT INFORMATION
# ============================================================

num_joints = p.getNumJoints(robot_id)

print("\n========================================")
print("ROBOT JOINT INFORMATION")
print("========================================")

print("Number of joints:", num_joints)

for joint_index in range(num_joints):

    info = p.getJointInfo(
        robot_id,
        joint_index
    )

    joint_name = info[1].decode("utf-8")
    joint_type = info[2]

    joint_axis = info[13]
    joint_origin = info[14]
    joint_orientation = info[15]

    parent_index = info[16]

    print(f"\nJoint index : {joint_index}")
    print(f"Name        : {joint_name}")
    print(f"Type        : {joint_type}")
    print(f"Parent index: {parent_index}")
    print(f"Axis        : {joint_axis}")
    print(f"Origin      : {joint_origin}")
    print(f"Orientation : {joint_orientation}")


# ============================================================
# CAMERA
# ============================================================

p.resetDebugVisualizerCamera(
    cameraDistance=0.55,
    cameraYaw=45,
    cameraPitch=-25,
    cameraTargetPosition=[0, 0, 0.15]
)


def set_camera(view):
    if view == 1:
        # Front
        p.resetDebugVisualizerCamera(
            cameraDistance=0.6,
            cameraYaw=90,
            cameraPitch=-10,
            cameraTargetPosition=[0, 0, 0.15]
        )

    elif view == 2:
        # Side
        p.resetDebugVisualizerCamera(
            cameraDistance=0.6,
            cameraYaw=0,
            cameraPitch=-10,
            cameraTargetPosition=[0, 0, 0.15]
        )

    elif view == 3:
        # Top
        p.resetDebugVisualizerCamera(
            cameraDistance=0.6,
            cameraYaw=90,
            cameraPitch=-89,
            cameraTargetPosition=[0, 0, 0.15]
        )

    elif view == 4:
        # Isometric
        p.resetDebugVisualizerCamera(
            cameraDistance=0.65,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.15]
        )


# ============================================================
# RUN SIMULATION
# ============================================================

print("\n========================================")
print("PyBullet is running.")
print("Close the PyBullet window to exit.")
print("========================================\n")

try:

    while p.isConnected():

        # We are NOT moving the joints yet.
        # This stage is only for inspecting the robot.

        p.stepSimulation()

        time.sleep(1.0 / 240.0)

        keys = p.getKeyboardEvents()

        if ord('1') in keys:
            set_camera(1)

        if ord('2') in keys:
            set_camera(2)

        if ord('3') in keys:
            set_camera(3)

        if ord('4') in keys:
            set_camera(4)


except KeyboardInterrupt:

    print("\nSimulation stopped by user.")


finally:

    if p.isConnected():
        p.disconnect()

    print("PyBullet disconnected.")