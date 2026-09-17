import numpy as np
import math

# ============================================================
# CORRECTED ROBOT GEOMETRY
# ============================================================
# All lengths are in metres.
#
# These constants come from the validated CAD/URDF geometry.
#
# phi = orientation of the J2 axis in the XY plane.
# c   = small horizontal offset from J1 axis to J2 axis.
# b   = J2-axis horizontal offset component.
# a   = horizontal component of J2 -> J3.
# h   = vertical component of J2 -> J3.
# H   = absolute height of J3 in the zero-angle reference.
#
# J3 does not change the gripper-link origin position because
# its axis passes through that link-frame origin.

PHI = math.radians(12.3929)

C = 0.009
B = 0.04044418285
A = 0.1159908533
H = 0.15768605725
HZ = 0.03075909567

# Exact resultant length of the J2 -> J3 position vector.
R = math.sqrt(A**2 + HZ**2)
DELTA = math.atan2(HZ, A)


def forward_kinematics(theta1, theta2, theta3=0.0):
    """
    Position FK for the corrected CAD-aligned model.

    Inputs:
        theta1, theta2, theta3 : degrees

    Returns:
        position : numpy array [x, y, z] in metres

    Note:
        theta3 affects orientation, but not the gripper-link origin
        position for this robot.
    """
    q1 = math.radians(theta1)
    q2 = math.radians(theta2)

    psi = q1 + PHI

    horizontal_component = (
        B - A * math.cos(q2) - HZ * math.sin(q2)
    )

    x = C * math.cos(psi) - horizontal_component * math.sin(psi)
    y = C * math.sin(psi) + horizontal_component * math.cos(psi)
    z = H - A * math.sin(q2) + HZ * math.cos(q2)

    position = np.array([x, y, z], dtype=float)

    # A compact 4x4 transform for the mathematical joint frame.
    # The exact physical mesh orientation also contains the fixed
    # CAD visual-frame rotation from the URDF.
    T03 = np.eye(4)
    T03[:3, 3] = position

    return T03, T03[:3, :3], position


if __name__ == "__main__":
    theta1 = 30.0
    theta2 = 45.0
    theta3 = -20.0

    T03, R03, position = forward_kinematics(theta1, theta2, theta3)

    print("\nCORRECTED FORWARD KINEMATICS")
    print("=" * 50)
    print(f"J1 = {theta1:.2f}°")
    print(f"J2 = {theta2:.2f}°")
    print(f"J3 = {theta3:.2f}°")

    print("\nEnd-effector position:")
    print(f"x = {position[0]:.6f} m")
    print(f"y = {position[1]:.6f} m")
    print(f"z = {position[2]:.6f} m")
