"""
Simple MuJoCo viewer for inspecting joint positions with gravity compensation.
Uses mjpython's built-in interactive viewer - drag the arm to see joint values.

Run with mjpython:
    mjpython joint_viewer.py
"""
import mujoco
import mujoco.viewer
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), "model", "kinova.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Set initial joint positions (modify these to change starting pose)
initial_qpos = [0, 0, 0, 0, 0, 0, 0]
data.qpos[:7] = initial_qpos
mujoco.mj_forward(model, data)

print("=" * 50)
print("Kinova Joint Viewer (with gravity compensation)")
print("=" * 50)
print("Drag the arm in the viewer to move joints.")
print("Joint positions are printed continuously.")
print("Close the viewer window to quit.")
print("=" * 50)

def print_joint_positions(data):
    """Print current joint positions in a readable format."""
    joint_names = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]
    positions = data.qpos[:7]
    
    # Clear line and print
    print("\rJoints: " + " | ".join(
        f"{name}: {pos:+.3f}" for name, pos in zip(joint_names, positions)
    ), end="", flush=True)

def apply_gravity_compensation(model, data):
    """Apply torques to counteract gravity, preserving external forces and velocities."""
    # Save state that we need to restore
    xfrc_applied_save = data.xfrc_applied.copy()
    qvel_save = data.qvel.copy()
    
    # Zero out external forces and velocities for inverse dynamics calculation
    data.xfrc_applied[:] = 0
    data.qvel[:] = 0
    data.qacc[:] = 0  # Zero acceleration
    
    # Compute gravity compensation torques
    mujoco.mj_inverse(model, data)
    gravity_torques = data.qfrc_inverse[:7].copy()
    
    # Restore external forces and velocities
    data.xfrc_applied[:] = xfrc_applied_save
    data.qvel[:] = qvel_save
    
    # Apply gravity compensation
    data.ctrl[:7] = gravity_torques

# Launch interactive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # Apply gravity compensation
        apply_gravity_compensation(model, data)
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Print joint positions
        print_joint_positions(data)
        
        # Sync viewer
        viewer.sync()

print("\n\nFinal joint positions:")
print(f"qpos = {list(np.round(data.qpos[:7], 4))}")
