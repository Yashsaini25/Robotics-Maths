import numpy as np


# ============================================================
# ROBOT PARAMETERS
# ============================================================

# Enter your actual CAD measurements later
L2 = None              # Distance: J2 axis → J3 axis
L3 = None              # Distance: J3 axis → end-effector target point
J2_OFFSET = None       # Height of J2 axis from the base coordinate origin


# ============================================================
# JOINT LIMITS
# ============================================================

# Enter your actual servo/joint limits later
# Values are in DEGREES

THETA1_MIN = None
THETA1_MAX = None

THETA2_MIN = None
THETA2_MAX = None

THETA3_MIN = None
THETA3_MAX = None


# ============================================================
# CHECK ROBOT PARAMETERS
# ============================================================

def check_parameters():

    parameters = {
        "L2": L2,
        "L3": L3,
        "J2_OFFSET": J2_OFFSET,

        "THETA1_MIN": THETA1_MIN,
        "THETA1_MAX": THETA1_MAX,

        "THETA2_MIN": THETA2_MIN,
        "THETA2_MAX": THETA2_MAX,

        "THETA3_MIN": THETA3_MIN,
        "THETA3_MAX": THETA3_MAX
    }

    missing = [
        name
        for name, value in parameters.items()
        if value is None
    ]

    if missing:
        raise ValueError(
            "The following robot parameters are not defined:\n"
            + ", ".join(missing)
        )


# ============================================================
# CHECK JOINT LIMIT
# ============================================================

def within_limit(angle_deg, minimum, maximum):

    return minimum <= angle_deg <= maximum


# ============================================================
# INVERSE KINEMATICS
# ============================================================

def inverse_kinematics(x, y, z):

    # Make sure robot parameters exist
    check_parameters()


    # --------------------------------------------------------
    # 1. JOINT 1
    # --------------------------------------------------------

    # Base rotation required to point toward target
    theta1 = np.arctan2(y, x)


    # --------------------------------------------------------
    # 2. HORIZONTAL DISTANCE
    # --------------------------------------------------------

    r = np.sqrt(x**2 + y**2)


    # --------------------------------------------------------
    # 3. VERTICAL DISTANCE
    # --------------------------------------------------------

    s = z - J2_OFFSET


    # --------------------------------------------------------
    # 4. DIRECT DISTANCE FROM J2 TO TARGET
    # --------------------------------------------------------

    D = np.sqrt(r**2 + s**2)


    # --------------------------------------------------------
    # 5. REACHABILITY CHECK
    # --------------------------------------------------------

    maximum_reach = L2 + L3
    minimum_reach = abs(L2 - L3)

    if D > maximum_reach:

        raise ValueError(
            "Target is outside the maximum reachable workspace."
        )

    if D < minimum_reach:

        raise ValueError(
            "Target is inside the minimum reachable workspace."
        )


    # --------------------------------------------------------
    # 6. CALCULATE ELBOW ANGLE
    # --------------------------------------------------------

    cos_theta3 = (
        L2**2 + L3**2 - D**2
    ) / (2 * L2 * L3)

    # Protect against tiny floating-point errors
    cos_theta3 = np.clip(
        cos_theta3,
        -1.0,
        1.0
    )

    theta3 = np.arccos(cos_theta3)


    # --------------------------------------------------------
    # 7. ANGLE FROM J2 TO TARGET
    # --------------------------------------------------------

    phi = np.arctan2(s, r)


    # --------------------------------------------------------
    # 8. ELBOW CORRECTION ANGLE
    # --------------------------------------------------------

    beta = np.arctan2(
        L3 * np.sin(theta3),
        L2 - L3 * np.cos(theta3)
    )


    # --------------------------------------------------------
    # 9. TWO POSSIBLE CONFIGURATIONS
    # --------------------------------------------------------

    # Elbow-up
    theta2_up = phi + beta
    theta3_up = theta3

    # Elbow-down
    theta2_down = phi - beta
    theta3_down = -theta3


    # --------------------------------------------------------
    # 10. CONVERT RADIANS → DEGREES
    # --------------------------------------------------------

    theta1_deg = np.degrees(theta1)

    theta2_up_deg = np.degrees(theta2_up)
    theta3_up_deg = np.degrees(theta3_up)

    theta2_down_deg = np.degrees(theta2_down)
    theta3_down_deg = np.degrees(theta3_down)


    # --------------------------------------------------------
    # 11. CREATE SOLUTIONS
    # --------------------------------------------------------

    elbow_up = (
        theta1_deg,
        theta2_up_deg,
        theta3_up_deg
    )

    elbow_down = (
        theta1_deg,
        theta2_down_deg,
        theta3_down_deg
    )


    # --------------------------------------------------------
    # 12. CHECK JOINT LIMITS
    # --------------------------------------------------------

    valid_solutions = []


    # =========================
    # ELBOW UP
    # =========================

    if (
        within_limit(
            theta1_deg,
            THETA1_MIN,
            THETA1_MAX
        )
        and
        within_limit(
            theta2_up_deg,
            THETA2_MIN,
            THETA2_MAX
        )
        and
        within_limit(
            theta3_up_deg,
            THETA3_MIN,
            THETA3_MAX
        )
    ):

        valid_solutions.append(
            ("elbow_up", elbow_up)
        )


    # =========================
    # ELBOW DOWN
    # =========================

    if (
        within_limit(
            theta1_deg,
            THETA1_MIN,
            THETA1_MAX
        )
        and
        within_limit(
            theta2_down_deg,
            THETA2_MIN,
            THETA2_MAX
        )
        and
        within_limit(
            theta3_down_deg,
            THETA3_MIN,
            THETA3_MAX
        )
    ):

        valid_solutions.append(
            ("elbow_down", elbow_down)
        )


    # --------------------------------------------------------
    # 13. NO VALID SOLUTION
    # --------------------------------------------------------

    if len(valid_solutions) == 0:

        raise ValueError(
            "Target is geometrically reachable, "
            "but no IK solution satisfies the joint limits."
        )


    return valid_solutions


# ============================================================
# EXAMPLE
# ============================================================

if __name__ == "__main__":

    # Target position
    x = 0.0
    y = 0.0
    z = 0.0

    solutions = inverse_kinematics(
        x,
        y,
        z
    )

    print("\nValid IK Solutions:")

    for configuration, angles in solutions:

        theta1, theta2, theta3 = angles

        print("\nConfiguration:", configuration)

        print(f"θ1 = {theta1:.2f}°")
        print(f"θ2 = {theta2:.2f}°")
        print(f"θ3 = {theta3:.2f}°")