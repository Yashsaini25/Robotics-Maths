import numpy as np

def Rx(theta):
    s, c = np.sin(theta), np.cos(theta)
    return np.array([[1, 0, 0], [0, c, s], [0, -s, c]])

def Ry(theta):
    s, c = np.sin(theta), np.cos(theta)
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])

def Rz(theta):
    s, c = np.sin(theta), np.cos(theta)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])

def homogeneous_transform(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def apply_transform(T, p):
    p_homogeneous = np.append(p, 1)
    p_transformed = T @ p_homogeneous
    return p_transformed[:3]

def invert_transform(T):
    """Efficient inverse of a homogeneous transform, using R^-1 = R^T."""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


theta = np.radians(40)
T = homogeneous_transform(Rz(theta), np.array([2, 1, 0]))

T_inv_fast = invert_transform(T)
T_inv_general = np.linalg.inv(T)

print("Fast inverse:\n", np.round(T_inv_fast, 4))
print("\nGeneral inverse:\n", np.round(T_inv_general, 4))
print("\nMatch?", np.allclose(T_inv_fast, T_inv_general))
print("T @ T_inv = identity?", np.allclose(T @ T_inv_fast, np.eye(4)))
print(np.round(T @ T_inv_fast, 4))

import matplotlib.pyplot as plt

def plot_frame(ax, T, label='', length=0.5):
    """Draw a coordinate frame's axes at the pose given by T."""
    origin = T[:3, 3]
    R = T[:3, :3]
    colors = ['r', 'g', 'b']   # x=red, y=green, z=blue
    for i in range(3):
        axis_dir = R[:, i] * length      # i-th column = i-th rotated axis direction
        ax.quiver(*origin, *axis_dir, color=colors[i], linewidth=2)
    ax.text(*origin, label, fontsize=10)

fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1, 3]); ax.set_ylim([-1, 3]); ax.set_zlim([-1, 3])

# World frame at origin
plot_frame(ax, np.eye(4), label='World')

# A transformed frame: rotate 45° about Z, shift to (2,1,0)
T = homogeneous_transform(Rz(np.radians(45)), np.array([2, 1, 0]))
plot_frame(ax, T, label='Frame A')

ax.set_title('World frame vs. transformed Frame A')
plt.show()