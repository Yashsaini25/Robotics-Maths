import numpy as np

from inverse_kinematics_corrected import inverse_kinematics
from robot_pybullet.forward_kinematics import forward_kinematics


def verify_solution(configuration, angles, target):
    theta1, theta2, theta3 = angles

    _, _, calculated = forward_kinematics(
        theta1, theta2, theta3
    )

    target = np.array(target, dtype=float)
    error = calculated - target
    position_error = np.linalg.norm(error)

    print("\n" + "=" * 60)
    print(f"Configuration: {configuration}")
    print("=" * 60)

    print("\nIK angles:")
    print(f"J1 = {theta1:.6f}°")
    print(f"J2 = {theta2:.6f}°")
    print(f"J3 = {theta3:.6f}°")

    print("\nTarget position:")
    print(f"x = {target[0]:.9f} m")
    print(f"y = {target[1]:.9f} m")
    print(f"z = {target[2]:.9f} m")

    print("\nFK calculated position:")
    print(f"x = {calculated[0]:.9f} m")
    print(f"y = {calculated[1]:.9f} m")
    print(f"z = {calculated[2]:.9f} m")

    print("\nPosition error:")
    print(f"dx = {error[0]:.12f} m")
    print(f"dy = {error[1]:.12f} m")
    print(f"dz = {error[2]:.12f} m")
    print(f"Total = {position_error:.12e} m")

    return position_error


if __name__ == "__main__":

    # A point generated from the already validated FK:
    # J1=30°, J2=45°, J3=-20°.
    target_angles = (30.0, 45.0, -20.0)
    _, _, target = forward_kinematics(*target_angles)

    print("\nIK → FK CLOSED-LOOP TEST")
    print("=" * 60)
    print("Original angles used to generate the target:")
    print(f"J1 = {target_angles[0]:.2f}°")
    print(f"J2 = {target_angles[1]:.2f}°")
    print(f"J3 = {target_angles[2]:.2f}°")

    print("\nGenerated target:")
    print(f"x = {target[0]:.9f} m")
    print(f"y = {target[1]:.9f} m")
    print(f"z = {target[2]:.9f} m")

    solutions = inverse_kinematics(*target)

    print(f"\nIK returned {len(solutions)} solution(s).")

    errors = []
    for configuration, angles in solutions:
        errors.append(
            verify_solution(configuration, angles, target)
        )

    print("\n" + "=" * 60)
    print("FINAL RESULT")
    print("=" * 60)

    if min(errors) < 1e-8:
        print("PASS: IK and FK are consistent.")
    else:
        print("CHECK: position error is larger than expected.")
