import os
import time
import math
import numpy as np
import pybullet as p
import pybullet_data

# ============================================================
# FK VALIDATION: ACTUAL URDF MATH vs PYBULLET
# ============================================================
# This version uses the JOINT ORIGINS and AXES that are actually
# present in the corrected robot.urdf.
#
# Test configuration:
#   J1 = 30 deg
#   J2 = 45 deg
#   J3 = -20 deg
#
# The previous validation script used the earlier Fusion-frame
# values (-Y for J2, -Z for J3). The corrected URDF uses the
# following actual parent-link joint frames:
#
# J1 origin = (0, 0, 0.110000)
# J1 axis   = (0, 0, 1)
#
# J2 origin = (0.00011040219, 0.04143331674, 0.04768605725)
# J2 axis   = (0.976698944, 0.214614011, 0)
#
# J3 origin = (0.02489326226, -0.11328814381, 0.03075909567)
# J3 axis   = (-0.976698944, -0.214614011, 0)
#
# We calculate:
#
# T = T(J1 origin) R(axis1,q1)
#     T(J2 origin) R(axis2,q2)
#     T(J3 origin) R(axis3,q3)
#
# and compare the resulting gripper link-frame position with
# PyBullet's getLinkState().
# ============================================================

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(PROJECT_DIR, "robot.urdf")

J1_DEG = 30.0
J2_DEG = 45.0
J3_DEG = -20.0

q = [math.radians(a) for a in [J1_DEG, J2_DEG, J3_DEG]]


def translation(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)

    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1.0 - c

    R = np.array([
        [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s,   z*y*C + x*s, c + z*z*C]
    ])

    T = np.eye(4)
    T[:3, :3] = R
    return T


def print_position(label, pos):
    print(
        f"{label}: "
        f"x={pos[0]: .6f} m, "
        f"y={pos[1]: .6f} m, "
        f"z={pos[2]: .6f} m"
    )


# ------------------------------------------------------------
# 1. Mathematical FK using the ACTUAL URDF values
# ------------------------------------------------------------

joint_origins = [
    (0.0, 0.0, 0.110000),
    (0.00011040219, 0.04143331674, 0.04768605725),
    (0.02489326226, -0.11328814381, 0.03075909567),
]

joint_axes = [
    (0.0, 0.0, 1.0),
    (0.976698944, 0.214614011, 0.0),
    (-0.976698944, -0.214614011, 0.0),
]

T = np.eye(4)

for origin, axis, angle in zip(joint_origins, joint_axes, q):
    T = T @ translation(*origin) @ axis_rotation(axis, angle)

fk_position = T[:3, 3]


# ------------------------------------------------------------
# 2. Start PyBullet
# ------------------------------------------------------------

if not os.path.isfile(URDF_PATH):
    raise FileNotFoundError(
        f"Could not find robot.urdf at:\n{URDF_PATH}"
    )

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")

robot_id = p.loadURDF(
    URDF_PATH,
    basePosition=[0, 0, 0],
    useFixedBase=True
)


# ------------------------------------------------------------
# 3. Find joints and gripper link
# ------------------------------------------------------------

expected = {
    "J1": "joint1_base_rotation",
    "J2": "joint2_shoulder",
    "J3": "joint3_wrist",
}

joint_indices = {}
gripper_link_index = None

for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)

    joint_name = info[1].decode("utf-8")
    child_link_name = info[12].decode("utf-8")

    for label, expected_name in expected.items():
        if joint_name == expected_name:
            joint_indices[label] = i

    if child_link_name == "gripper":
        gripper_link_index = i

missing = [name for name in expected if name not in joint_indices]

if missing:
    raise RuntimeError(f"Missing joints: {missing}")

if gripper_link_index is None:
    raise RuntimeError("Could not find the gripper link.")


# ------------------------------------------------------------
# 4. Apply exactly the same joint angles
# ------------------------------------------------------------

for label, angle in zip(["J1", "J2", "J3"], q):
    p.resetJointState(robot_id, joint_indices[label], angle)

p.stepSimulation()


# ------------------------------------------------------------
# 5. Read PyBullet gripper link frame
# ------------------------------------------------------------

state = p.getLinkState(
    robot_id,
    gripper_link_index,
    computeForwardKinematics=True
)

# PyBullet's link frame position.
pybullet_position = np.array(state[4])


# ------------------------------------------------------------
# 6. Compare
# ------------------------------------------------------------

difference = pybullet_position - fk_position
error = np.linalg.norm(difference)

print("\n========================================")
print("FK VALIDATION")
print("========================================")

print(
    f"Test angles: "
    f"J1={J1_DEG:.2f}°, "
    f"J2={J2_DEG:.2f}°, "
    f"J3={J3_DEG:.2f}°"
)

print("\nMATHEMATICAL FK")
print_position("Gripper link frame", fk_position)

print("\nPYBULLET")
print_position("Gripper link frame", pybullet_position)

print("\nDIFFERENCE (PyBullet - FK)")
print_position("Difference", difference)

print(
    f"\nPosition error: "
    f"{error:.9f} m "
    f"({error * 1000:.6f} mm)"
)

if error < 1e-9:
    print("\nRESULT: EXACT FK MATCH (within numerical precision).")
elif error < 1e-6:
    print("\nRESULT: FK MATCHES PYBULLET within 0.001 mm.")
elif error < 1e-4:
    print("\nRESULT: FK MATCHES PYBULLET within 0.1 mm.")
else:
    print("\nRESULT: FK MISMATCH — inspect the URDF frames.")


# ------------------------------------------------------------
# 7. Camera
# ------------------------------------------------------------

p.resetDebugVisualizerCamera(
    cameraDistance=0.55,
    cameraYaw=45,
    cameraPitch=-25,
    cameraTargetPosition=[0, 0, 0.15]
)

print("\nPyBullet is running. Close the window to exit.")

try:
    while p.isConnected():
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

except KeyboardInterrupt:
    pass

finally:
    if p.isConnected():
        p.disconnect()
