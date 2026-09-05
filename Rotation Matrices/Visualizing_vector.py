import matplotlib.pyplot as plt
import numpy as np

def Rz(theta):
    s, c = np.sin(theta), np.cos(theta)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])

# Original vector (e.g. pointing along x-axis, slightly up)
v = np.array([1, 0, 0.3])

# Rotate it by 60 degrees about Z
theta = np.radians(60)
R = Rz(theta)
v_rotated = R @ v

fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection='3d')

# Draw original and rotated vectors as arrows from origin
ax.quiver(0, 0, 0, *v, color='blue', label='Original vector', linewidth=2)
ax.quiver(0, 0, 0, *v_rotated, color='red', label='Rotated vector (60° about Z)', linewidth=2)

# Draw reference axes for context
ax.quiver(0, 0, 0, 1, 0, 0, color='gray', alpha=0.3)
ax.quiver(0, 0, 0, 0, 1, 0, color='gray', alpha=0.3)
ax.quiver(0, 0, 0, 0, 0, 1, color='gray', alpha=0.3)

ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 1.5])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
ax.set_title('Vector before and after rotation about Z-axis')

plt.show()

