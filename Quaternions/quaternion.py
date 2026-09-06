import numpy as np

def quat_from_axis_angle(axis, theta):
    axis = axis / np.linalg.norm(axis)

    x, y, z = axis
    w = np.cos(theta / 2)
    x = x * np.sin(theta / 2)
    y = y * np.sin(theta / 2)
    z = z * np.sin(theta / 2)

    return np.array([w, x, y, z])


def quat_conjugate(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w, x, y, z])


def rotate_vector_by_quaternion(v, q):
    q_conj = quat_conjugate(q)
    v_quat = np.array([0, *v])
    rotated_v_quat = quat_multiply(quat_multiply(q, v_quat), q_conj)
    return rotated_v_quat[1:]  


def Rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def quaternion_to_rotation(q):
    w, x, y, z = q
    R = np.array([
        [1 - 2*(y**2 + z**2),  2*(x*y - w*z),      2*(x*z + w*y)],
        [2*(x*y + w*z),        1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y),        2*(y*z + w*x),      1 - 2*(x**2 + y**2)]
    ])

    return R


# Rotate 60 degrees about Z-axis
theta = np.radians(60)
axis = np.array([0, 0, 1])
p = np.array([1, 0, 0.3])

# Method 1: rotation matrix
R = Rz(theta)
p_matrix = R @ p

# Method 2: quaternion
q = quat_from_axis_angle(axis, theta)
p_quat = rotate_vector_by_quaternion(p, q)

quaternion_rotation_matrix = quaternion_to_rotation(q)

print("Rotation matrix result:", p_matrix)
print("Quaternion result:     ", p_quat)
print("Match?", np.allclose(p_matrix, p_quat))
print("Match with quaternion rotation matrix?", np.allclose(p_matrix, quaternion_rotation_matrix @ p))