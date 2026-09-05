import numpy as np

def Rx(theta):
    s, c = np.sin(theta), np.cos(theta)
    return np.array([[1,0,0], [0, c, s], [0,-s, c]])

def Ry(theta):
    s, c = np.sin(theta), np.cos(theta)
    return np.array([[c, 0,-s], [0, 1, 0], [s, 0, c]])

def Rz(theta):
    s, c = np.sin(theta), np.cos(theta)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])

alpha = np.radians(30)
beta = np.radians(45)
gamma = np.radians(60)

R_euler = Rz(gamma) @ Ry(beta) @ Rx(alpha)
R_fixed = Rz(gamma) @ Ry(beta) @ Rx(alpha)

print("R (ZYX Euler) =\n", np.round(R_euler, 4))
print("\nR (XYZ Fixed) =\n", np.round(R_fixed, 4))
print("\nAre they equal? ", np.allclose(R_euler, R_fixed))