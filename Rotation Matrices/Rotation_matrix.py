import numpy as np

def rotation_x(theta):
    return np.array([[1, 0,              0],
                     [0, np.cos(theta),  np.sin(theta)],
                     [0, -np.sin(theta), np.cos(theta)]])

def rotation_y(theta):
    return np.array([[np.cos(theta), 0, -np.sin(theta)],
                     [0,             1, 0],
                     [np.sin(theta), 0, np.cos(theta)]])

def rotation_z(theta):
    return np.array([[np.cos(theta),  np.sin(theta), 0],
                     [-np.sin(theta), np.cos(theta), 0],
                     [0,              0,             1]])

theta = np.deg2rad(90)

R = rotation_z(theta)

# Original X-axis
x_axis = np.array([1, 0, 0])

# Rotate X-axis
rotated_x = R @ x_axis

print("Rotation matrix:")
print(R)

print("\nOriginal X-axis:")
print(x_axis)

print("\nRotated X-axis:")
print(rotated_x)

