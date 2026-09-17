import numpy as np
import math

# ============================================================
# CORRECTED ROBOT GEOMETRY
# ============================================================
# All lengths are in metres.

PHI = math.radians(12.3929)

C = 0.009
B = 0.04044418285
A = 0.1159908533
H = 0.15768605725
HZ = 0.03075909567

R = math.sqrt(A**2 + HZ**2)
DELTA = math.atan2(HZ, A)


def inverse_kinematics(x, y, z):
    """
    Position-only IK for the corrected robot.

    Inputs:
        x, y, z : target position in metres

    Returns:
        A list of (configuration, (theta1, theta2, theta3)).

    Important:
        Position alone cannot uniquely determine theta3 for this robot,
        because J3 rotates about an axis passing through the gripper-link
        origin. Therefore theta3 is returned as 0° here. A desired
        end-effector orientation can later be used to solve theta3.
    """

    x = float(x)
    y = float(y)
    z = float(z)

    # --------------------------------------------------------
    # 1. Distance from the base axis to the target
    # --------------------------------------------------------
    r = math.hypot(x, y)

    if r < C:
        raise ValueError(
            "Target horizontal radius is too small for the fixed "
            "J1-to-J2 offset."
        )

    # --------------------------------------------------------
    # 2. The geometry gives:
    #
    #       r^2 = C^2 + B(q2)^2
    #
    # Therefore there are two possible horizontal branches.
    # --------------------------------------------------------
    B_abs = math.sqrt(max(0.0, r*r - C*C))

    B_candidates = [B_abs]
    if B_abs > 1e-10:
        B_candidates.append(-B_abs)

    solutions = []

    # --------------------------------------------------------
    # 3. Solve each branch
    # --------------------------------------------------------
    for branch_index, Bq in enumerate(B_candidates):

        # From:
        #   Bq - B = -R*cos(theta2 - DELTA)
        #   H - z =  R*sin(theta2 - DELTA)
        #
        # hence:
        #   theta2 - DELTA = atan2(H-z, B-Bq)
        # The point must lie on the circle traced by J3 as J2 rotates:
        #
        #   (Bq - B)^2 + (z - H)^2 = R^2
        #
        # Reject a branch that is not geometrically consistent.
        circle_error = (
            (Bq - B)**2 + (z - H)**2 - R**2
        )

        if abs(circle_error) > 1e-6:
            continue

        theta2 = DELTA + math.atan2(H - z, B - Bq)

        # Target azimuth:
        #
        # atan2(y,x) = (theta1 + PHI) + atan2(Bq,C)
        #
        target_angle = math.atan2(y, x)
        theta1 = (
            target_angle
            - PHI
            - math.atan2(Bq, C)
        )

        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)

        # Normalize to [-180, 180)
        theta1_deg = (theta1_deg + 180.0) % 360.0 - 180.0
        theta2_deg = (theta2_deg + 180.0) % 360.0 - 180.0

        configuration = (
            "branch_1" if branch_index == 0 else "branch_2"
        )

        solutions.append(
            (configuration, (theta1_deg, theta2_deg, 0.0))
        )

    if not solutions:
        raise ValueError(
            "Target is outside the robot's position workspace, "
            "or it does not lie on the reachable kinematic surface."
        )

    return solutions


if __name__ == "__main__":
    target = (0.04934037, -0.04069913, 0.09741810)

    print("\nCORRECTED INVERSE KINEMATICS")
    print("=" * 50)
    print(f"Target: x={target[0]:.6f}, "
          f"y={target[1]:.6f}, z={target[2]:.6f}")

    for configuration, angles in inverse_kinematics(*target):
        print(f"\n{configuration}:")
        print(f"J1 = {angles[0]:.6f}°")
        print(f"J2 = {angles[1]:.6f}°")
        print(f"J3 = {angles[2]:.6f}°")
