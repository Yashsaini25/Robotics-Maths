import numpy as np

from inverse_kinematics import inverse_kinematics
from forward_kinematics import forward_kinematics

TARGET_X = None
TARGET_Y = None
TARGET_Z = None


def verify_solution(configuration, angles, target):
    theta1, theta2, theta3 = angles

    T03, R03, calculated_position = forward_kinematics(
        theta1, theta2, theta3
    )

    target_position = np.array(target)
    error = calculated_position - target_position
    position_error = np.linalg.norm(error)

    print("\n" + "=" * 50)
    print(f"Configuration: {configuration}")
    print("=" * 50)

    print("\nJoint angles from IK:")
    print(f"θ1 = {theta1:.3f}°")
    print(f"θ2 = {theta2:.3f}°")
    print(f"θ3 = {theta3:.3f}°")

    print("\nRequested target:")
    print(f"x = {target_position[0]:.3f}")
    print(f"y = {target_position[1]:.3f}")
    print(f"z = {target_position[2]:.3f}")

    print("\nFK calculated position:")
    print(f"x = {calculated_position[0]:.3f}")
    print(f"y = {calculated_position[1]:.3f}")
    print(f"z = {calculated_position[2]:.3f}")

    print("\nPosition error:")
    print(f"Δx = {error[0]:.6f}")
    print(f"Δy = {error[1]:.6f}")
    print(f"Δz = {error[2]:.6f}")
    print(f"\nTotal position error = {position_error:.6f}")

    return position_error


if __name__ == "__main__":
    if TARGET_X is None or TARGET_Y is None or TARGET_Z is None:
        raise ValueError(
            "Enter TARGET_X, TARGET_Y and TARGET_Z before running."
        )

    target = (TARGET_X, TARGET_Y, TARGET_Z)

    solutions = inverse_kinematics(
        TARGET_X, TARGET_Y, TARGET_Z
    )

    print("\nIK solutions found:")

    for configuration, angles in solutions:
        verify_solution(configuration, angles, target)
