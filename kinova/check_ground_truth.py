"""
Compute ground truth composite inertial parameters for link6+link7.
Compare against system ID estimates.
"""
import numpy as np

# From kinova_fullinertia.xml:
# Link6: mass=0.25081, pos=(-9.215e-08, 0.0034219, 0.04541)
#        fullinertia=(Ixx, Iyy, Izz, Ixy, Ixz, Iyz) = (2.1273e-04, 2.26129748e-04, 1.37770251e-04, -1.46736533e-09, 2.76081180e-09, -3.01568714e-05)
#
# Link7: mass=0.53835, pos=(-8.73971e-05, -0.0847593, 0.0026466) in link7 frame
#        fullinertia = (1.43431024e-03, 2.84530905e-04, 1.48006585e-03, 4.58717725e-07, 1.90849247e-07, -9.87620436e-05)
#        Link7 is at pos=(0, 0.01265, 0.1178) with quat=(0.707105, -0.707108, 0, 0) relative to link6

# Link6 parameters in link6 frame
m6 = 0.25081
c6 = np.array([-9.215e-08, 0.0034219, 0.04541])
# fullinertia order: Ixx, Iyy, Izz, Ixy, Ixz, Iyz
I6 = np.array([
    [2.1273e-04, -1.46736533e-09, 2.76081180e-09],
    [-1.46736533e-09, 2.26129748e-04, -3.01568714e-05],
    [2.76081180e-09, -3.01568714e-05, 1.37770251e-04]
])

# Link7 parameters in link7 frame
m7 = 0.53835
c7_local = np.array([-8.73971e-05, -0.0847593, 0.0026466])
I7_local = np.array([
    [1.43431024e-03, 4.58717725e-07, 1.90849247e-07],
    [4.58717725e-07, 2.84530905e-04, -9.87620436e-05],
    [1.90849247e-07, -9.87620436e-05, 1.48006585e-03]
])

# Transform link7 to link6 frame
# Link7 position relative to link6: (0, 0.01265, 0.1178)
# Link7 orientation: quat=(0.707105, -0.707108, 0, 0) which is 90° rotation about X
p7_in_6 = np.array([0, 0.01265, 0.1178])

# Quaternion (w, x, y, z) = (0.707105, -0.707108, 0, 0) -> rotation matrix
# This is a -90° rotation about X axis
theta = -np.pi/2
R7_to_6 = np.array([
    [1, 0, 0],
    [0, np.cos(theta), -np.sin(theta)],
    [0, np.sin(theta), np.cos(theta)]
])

print("R7_to_6 (link7 to link6 rotation):")
print(R7_to_6)

# Transform link7 CoM to link6 frame
c7_in_6 = p7_in_6 + R7_to_6 @ c7_local
print(f"\nLink7 CoM in link6 frame: {c7_in_6}")

# Transform link7 inertia to link6 frame (rotate)
I7_in_6_at_c7 = R7_to_6 @ I7_local @ R7_to_6.T

# Parallel axis theorem to move from link7 CoM to link6 origin
def parallel_axis(I_com, m, r):
    """Move inertia from CoM to point at offset r."""
    # I_new = I_com + m * (|r|^2 * eye(3) - outer(r, r))
    return I_com + m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))

I7_in_6_at_origin = parallel_axis(I7_in_6_at_c7, m7, c7_in_6)

# Also need to move link6 inertia from its CoM to origin
I6_at_origin = parallel_axis(I6, m6, c6)

# Composite inertia at link6 origin
I_composite = I6_at_origin + I7_in_6_at_origin

# Composite mass
m_composite = m6 + m7

# Composite CoM
c_composite = (m6 * c6 + m7 * c7_in_6) / m_composite

# First moment h = m * c
h_composite = m_composite * c_composite

print("\n=== Ground Truth Composite Parameters (link6 + link7) ===")
print(f"m  = {m_composite:.6f}")
print(f"hx = {h_composite[0]:.6f}")
print(f"hy = {h_composite[1]:.6f}")
print(f"hz = {h_composite[2]:.6f}")
print(f"Ixx = {I_composite[0,0]:.6f}")
print(f"Ixy = {I_composite[0,1]:.6f}")
print(f"Ixz = {I_composite[0,2]:.6f}")
print(f"Iyy = {I_composite[1,1]:.6f}")
print(f"Iyz = {I_composite[1,2]:.6f}")
print(f"Izz = {I_composite[2,2]:.6f}")

print("\n=== Identifiable combinations ===")
print(f"hx = {h_composite[0]:.6f}")
print(f"hz = {h_composite[2]:.6f}")
print(f"Iyy = {I_composite[1,1]:.6f}")
print(f"Ixy = {I_composite[0,1]:.6f}")
print(f"Ixz = {I_composite[0,2]:.6f}")
print(f"Iyz = {I_composite[1,2]:.6f}")
print(f"Ixx + Izz = {I_composite[0,0] + I_composite[2,2]:.6f}")
