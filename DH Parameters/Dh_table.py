import numpy as np

def dh_transform(a, alpha, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa,  a * ct],
        [st,  ct * ca, -ct * sa,  a * st],
        [0,   sa,       ca,       d     ],
        [0,   0,        0,        1     ]
    ])

d1 = 0.5  # Example value for d1
L1 = 1.0  # Example value for L1
L2 = 0.5  # Example value for L2

def get_dh_table(theta1, theta2, theta3):
    return [
        (theta1, d1, 0, np.radians(90)),
        (theta2, 0, L1, 0),
        (theta3, 0, L2, 0)
    ]

def forward_kinematics(dh_table):
    T = np.eye(4)
    transforms = []

    for (theta, d, a, alpha) in dh_table:
        T_i = dh_transform(a, alpha, d, theta)
        T = T @ T_i
        transforms.append(T)

    return transforms

theta1 = np.radians(30)
theta2 = np.radians(45)
theta3 = np.radians(-20)

dh_table = get_dh_table(theta1, theta2, theta3)
transforms = forward_kinematics(dh_table)

joint_names = ['Base', 'Shoulder', 'Elbow (end-effector)']
for name, T in zip(joint_names, transforms):
    pos = T[:3, 3]
    print(f"{name}: position = {np.round(pos, 3)}")

import matplotlib.pyplot as plt

fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(111, projection='3d')

# Get all joint positions, including the base origin itself
positions = [np.array([0,0,0])] + [T[:3,3] for T in transforms]
positions = np.array(positions)

# Draw links as connected line segments
ax.plot(positions[:,0], positions[:,1], positions[:,2], '-o', color='black', linewidth=2, markersize=6)

# Label each joint
for name, pos in zip(['Base_origin'] + joint_names, positions):
    ax.text(*pos, name, fontsize=8)

ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.set_title('3-DOF arm pose from DH parameters')
plt.show()