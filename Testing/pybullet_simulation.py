import pybullet as p
import pybullet_data
import time


# ============================================================
# CONNECT TO PYBULLET
# ============================================================

physics_client = p.connect(p.GUI)

if physics_client < 0:
    raise RuntimeError("Could not connect to PyBullet")


# ============================================================
# PYBULLET SETTINGS
# ============================================================

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)


# ============================================================
# LOAD GROUND
# ============================================================

plane_id = p.loadURDF("plane.urdf")


# ============================================================
# LOAD TEST ROBOT
# ============================================================

robot_id = p.loadURDF(
    "r2d2.urdf",
    basePosition=[0, 0, 0.5]
)


# ============================================================
# SIMULATION LOOP
# ============================================================

print("PyBullet simulation started.")
print("Close the PyBullet window to exit.")

try:

    while p.isConnected():

        p.stepSimulation()

        time.sleep(1 / 240)

except KeyboardInterrupt:

    print("Simulation stopped.")


# ============================================================
# DISCONNECT
# ============================================================

if p.isConnected():
    p.disconnect()