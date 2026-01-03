"""Test Pinocchio with Kinova URDF"""
import numpy as np
import pinocchio as pin
from pathlib import Path

# Load URDF
urdf_path = Path(__file__).parent / "urdf" / "kinova.urdf"
model = pin.buildModelFromUrdf(str(urdf_path))
data = model.createData()

print(f"Model: {model.name}")
print(f"DOF: {model.nq}")
print(f"Joints: {[model.names[i] for i in range(model.njoints)]}")

# Test forward kinematics
q = np.zeros(model.nq)
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

# Get end-effector pose
ee_frame = model.getFrameId("link_tool")
if ee_frame < model.nframes:
    pose = data.oMf[ee_frame]
    print(f"\nEnd-effector position (q=0): {pose.translation}")

# Test dynamics
M = pin.crba(model, data, q)
g = pin.computeGeneralizedGravity(model, data, q)
print(f"\nGravity torques (q=0): {g}")
