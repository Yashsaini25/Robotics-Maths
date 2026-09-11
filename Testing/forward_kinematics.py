import numpy as np


# ============================================================
# ROBOT PARAMETERS
# ============================================================

# Enter your actual CAD/DH values later.
#
# a  = link length along X
# d  = link offset along Z
# alpha = link twist around X
#
# Units:
#   Lengths -> same unit throughout the program
#   Angles  -> degrees here, converted to radians internally


A1 = None
A2 = None
A3 = None

D1 = None
D2 = None
D3 = None

ALPHA1 = None
ALPHA2 = None
ALPHA3 = None


# ============================================================
# DH TRANSFORMATION MATRIX
# ============================================================

def dh_transform(theta, d, a, alpha):

    # Convert degrees to radians
    theta = np.radians(theta)
    alpha = np.radians(alpha)

    ct = np.cos(theta)
    st = np.sin(theta)

    ca = np.cos(alpha)
    sa = np.sin(alpha)

    T = np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,        sa,       ca,      d],
        [0,         0,        0,      1]
    ])

    return T


# ============================================================
# FORWARD KINEMATICS
# ============================================================

def forward_kinematics(theta1, theta2, theta3):

    # Make sure DH parameters have been entered
    parameters = {
        "A1": A1,
        "A2": A2,
        "A3": A3,
        "D1": D1,
        "D2": D2,
        "D3": D3,
        "ALPHA1": ALPHA1,
        "ALPHA2": ALPHA2,
        "ALPHA3": ALPHA3
    }

    missing = [
        name
        for name, value in parameters.items()
        if value is None
    ]

    if missing:
        raise ValueError(
            "The following DH parameters are not defined:\n"
            + ", ".join(missing)
        )


    # --------------------------------------------------------
    # Individual transformations
    # --------------------------------------------------------

    T01 = dh_transform(
        theta1,
        D1,
        A1,
        ALPHA1
    )

    T12 = dh_transform(
        theta2,
        D2,
        A2,
        ALPHA2
    )

    T23 = dh_transform(
        theta3,
        D3,
        A3,
        ALPHA3
    )


    # --------------------------------------------------------
    # Complete transformation
    # --------------------------------------------------------

    T03 = T01 @ T12 @ T23


    # --------------------------------------------------------
    # Extract end-effector position
    # --------------------------------------------------------

    x = T03[0, 3]
    y = T03[1, 3]
    z = T03[2, 3]


    # --------------------------------------------------------
    # Extract end-effector orientation
    # --------------------------------------------------------

    R03 = T03[0:3, 0:3]


    return T03, R03, np.array([x, y, z])


# ============================================================
# EXAMPLE
# ============================================================

if __name__ == "__main__":

    # Example joint angles
    theta1 = 0
    theta2 = 0
    theta3 = 0

    T03, R03, position = forward_kinematics(
        theta1,
        theta2,
        theta3
    )

    print("\nFinal Transformation Matrix T03:")
    print(T03)

    print("\nEnd-Effector Rotation Matrix R03:")
    print(R03)

    print("\nEnd-Effector Position:")
    print(f"x = {position[0]:.3f}")
    print(f"y = {position[1]:.3f}")
    print(f"z = {position[2]:.3f}")