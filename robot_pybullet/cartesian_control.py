import math
import time
import pybullet as p
import pybullet_data

from inverse_kinematics_corrected import inverse_kinematics


URDF_FILE = "robot.urdf"

JOINT_NAMES = {
    "joint1_base_rotation": None,
    "joint2_shoulder": None,
    "joint3_wrist": None,
}


def find_joints(robot_id):
    found = {}

    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        name = info[1].decode("utf-8")

        if name in JOINT_NAMES:
            found[name] = i

    missing = [name for name, index in found.items() if index is None]

    if len(found) != len(JOINT_NAMES):
        missing = [name for name in JOINT_NAMES if name not in found]
        raise RuntimeError(
            "Could not find joint(s): " + ", ".join(missing)
        )

    return found


def move_robot(robot_id, joint_indices, angles_deg):
    for name, angle_deg in zip(
        ["joint1_base_rotation", "joint2_shoulder", "joint3_wrist"],
        angles_deg,
    ):
        joint_index = joint_indices[name]
        angle_rad = math.radians(angle_deg)

        p.resetJointState(
            robot_id,
            joint_index,
            angle_rad
        )


def get_gripper_position(robot_id, joint_index):
    state = p.getLinkState(
        robot_id,
        joint_index,
        computeForwardKinematics=True
    )

    # state[4] = world position of the link frame
    return state[4]


def main():

    # ------------------------------------------------------------
    # Start PyBullet
    # ------------------------------------------------------------
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    robot_id = p.loadURDF(
        URDF_FILE,
        useFixedBase=True
    )

    joint_indices = find_joints(robot_id)

    gripper_joint_index = joint_indices["joint3_wrist"]

    print("\n" + "=" * 60)
    print("CARTESIAN POSITION CONTROL")
    print("=" * 60)
    print("Enter target coordinates in metres.")
    print("Example: 0.049 -0.041 0.097")
    print("Type 'q' to quit.\n")

    while True:

        user_input = input("Target x y z: ").strip()

        if user_input.lower() == "q":
            break

        try:
            values = user_input.replace(",", " ").split()

            if len(values) != 3:
                print("Please enter exactly three values: x y z")
                continue

            x, y, z = map(float, values)

            # ----------------------------------------------------
            # IK
            # ----------------------------------------------------
            solutions = inverse_kinematics(x, y, z)

            print("\nIK solutions:")

            for i, (configuration, angles) in enumerate(solutions, 1):
                theta1, theta2, theta3 = angles

                print(
                    f"{i}. {configuration}: "
                    f"J1={theta1:.2f}°, "
                    f"J2={theta2:.2f}°, "
                    f"J3={theta3:.2f}°"
                )

            # For now choose the first valid solution.
            configuration, angles = solutions[0]

            print(f"\nUsing: {configuration}")

            # ----------------------------------------------------
            # Move robot
            # ----------------------------------------------------
            move_robot(
                robot_id,
                joint_indices,
                angles
            )

            # Give PyBullet a moment to update FK.
            for _ in range(10):
                p.stepSimulation()
                time.sleep(0.01)

            actual_position = get_gripper_position(
                robot_id,
                gripper_joint_index
            )

            error = (
                actual_position[0] - x,
                actual_position[1] - y,
                actual_position[2] - z,
            )

            error_distance = math.sqrt(
                error[0] ** 2 +
                error[1] ** 2 +
                error[2] ** 2
            )

            print("\nRequested position:")
            print(f"x = {x:.6f} m")
            print(f"y = {y:.6f} m")
            print(f"z = {z:.6f} m")

            print("\nPyBullet gripper position:")
            print(f"x = {actual_position[0]:.6f} m")
            print(f"y = {actual_position[1]:.6f} m")
            print(f"z = {actual_position[2]:.6f} m")

            print("\nPosition error:")
            print(f"dx = {error[0]:.9f} m")
            print(f"dy = {error[1]:.9f} m")
            print(f"dz = {error[2]:.9f} m")
            print(f"total = {error_distance:.9f} m")

        except ValueError as e:
            print(f"\nIK error: {e}")

        except Exception as e:
            print(f"\nError: {e}")

    p.disconnect()
    print("\nPyBullet closed.")


if __name__ == "__main__":
    main()
