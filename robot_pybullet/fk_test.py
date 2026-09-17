import math
import numpy as np

# ============================================================
# FK TEST PROGRAM
# CAD/URDF-aligned mathematical forward kinematics
#
# Enter J1, J2, J3 in degrees.
# The program calculates:
#   1. End-effector/gripper-link position
#   2. Full 4x4 homogeneous transformation matrix
#   3. Gripper orientation as roll/pitch/yaw
#
# These equations use the same corrected URDF joint geometry
# that was already validated against PyBullet.
# ============================================================

# -----------------------------
# Robot geometry (metres)
# -----------------------------
phi = math.radians(12.3929)

c = 0.00900000000
b = 0.04044418285
a = 0.1159908533
h = 0.03075909567
H = 0.15768605725


def rot_z(angle):
    c_, s_ = math.cos(angle), math.sin(angle)
    return np.array([
        [c_, -s_, 0.0],
        [s_,  c_, 0.0],
        [0.0, 0.0, 1.0]
    ])


def rot_axis(axis, angle):
    """Rodrigues rotation matrix for an arbitrary unit axis."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)

    x, y, z = axis
    c_, s_ = math.cos(angle), math.sin(angle)
    C = 1.0 - c_

    return np.array([
        [c_ + x*x*C,     x*y*C - z*s_, x*z*C + y*s_],
        [y*x*C + z*s_,   c_ + y*y*C,   y*z*C - x*s_],
        [z*x*C - y*s_,   z*y*C + x*s_, c_ + z*z*C]
    ])


def make_T(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def fk(q1, q2, q3):
    """
    FK using the actual corrected URDF joint origins/axes.

    This is deliberately written as the general homogeneous
    transformation chain, so the result can later be compared
    directly with PyBullet.
    """

    # Actual URDF joint data
    p1 = np.array([0.0, 0.0, 0.110000])
    a1 = np.array([0.0, 0.0, 1.0])

    p2 = np.array([
        0.00011040219,
        0.04143331674,
        0.04768605725
    ])
    a2 = np.array([
        0.976698944,
        0.214614011,
        0.0
    ])

    p3 = np.array([
        0.02489326226,
        -0.11328814381,
        0.03075909567
    ])
    a3 = np.array([
        -0.976698944,
        -0.214614011,
        0.0
    ])

    # The URDF joint frames are parent-link frames.
    # Chain: translation to joint -> rotation about joint axis.
    T = np.eye(4)

    T = T @ make_T(np.eye(3), p1)
    T = T @ make_T(rot_axis(a1, q1), np.zeros(3))

    T = T @ make_T(np.eye(3), p2)
    T = T @ make_T(rot_axis(a2, q2), np.zeros(3))

    T = T @ make_T(np.eye(3), p3)
    T = T @ make_T(rot_axis(a3, q3), np.zeros(3))

    return T


def rpy_from_rotation(R):
    """
    Convert rotation matrix to roll, pitch, yaw (XYZ convention).
    Returned values are in degrees.
    """
    # Handle the normal non-singular case.
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)

    if sy > 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        # Gimbal-lock case
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return np.degrees([roll, pitch, yaw])


def print_matrix(T):
    for row in T:
        print("  " + "  ".join(f"{v: .6f}" for v in row))


print("=" * 60)
print("3-DOF ROBOT — MATHEMATICAL FK")
print("=" * 60)

while True:
    try:
        q1_deg = float(input("\nEnter J1 angle (deg), or 'q' to quit: "))
        q2_deg = float(input("Enter J2 angle (deg): "))
        q3_deg = float(input("Enter J3 angle (deg): "))
    except ValueError:
        print("Please enter numeric angles.")
        continue

    q1 = math.radians(q1_deg)
    q2 = math.radians(q2_deg)
    q3 = math.radians(q3_deg)

    T = fk(q1, q2, q3)

    position = T[:3, 3]
    orientation = rpy_from_rotation(T[:3, :3])

    print("\n" + "-" * 60)
    print("JOINT CONFIGURATION")
    print("-" * 60)
    print(f"J1 = {q1_deg: .3f} deg")
    print(f"J2 = {q2_deg: .3f} deg")
    print(f"J3 = {q3_deg: .3f} deg")

    print("\nGRIPPER LINK POSITION")
    print(f"x = {position[0]: .6f} m   ({position[0]*1000: .3f} mm)")
    print(f"y = {position[1]: .6f} m   ({position[1]*1000: .3f} mm)")
    print(f"z = {position[2]: .6f} m   ({position[2]*1000: .3f} mm)")

    print("\nHOMOGENEOUS TRANSFORMATION MATRIX T")
    print_matrix(T)

    print("\nGRIPPER ORIENTATION (RPY)")
    print(f"Roll  = {orientation[0]: .3f} deg")
    print(f"Pitch = {orientation[1]: .3f} deg")
    print(f"Yaw   = {orientation[2]: .3f} deg")

    print("\nNOTE:")
    print("J3 changes the gripper orientation but does not change")
    print("the gripper-link frame position in this kinematic model.")

    print("\n" + "=" * 60)
