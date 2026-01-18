"""
Simple MuJoCo viewer for inspecting joint positions with gravity compensation.
Uses mjpython's built-in interactive viewer - drag the arm to see joint values.
Now uses Motor class with 2ms delay for torque commands.

Run with mjpython:
    mjpython joint_viewer.py
"""
import mujoco
import mujoco.viewer
import numpy as np
import os
from motor import Motor

# Load model
model_path = os.path.join(os.path.dirname(__file__), "model", "kinova.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Create 7 motor objects (one for each joint)
# Using direct torque command mode (not MIT controller)
# Stribeck friction parameters estimated for Kinova arm joints
friction_params = [
    # J1, J2 (base/shoulder) - larger joints
    {"T_coulomb": 0.4, "T_static": 0.5, "omega_s": 0.05, "delta": 2},
    {"T_coulomb": 0.4, "T_static": 0.5, "omega_s": 0.05, "delta": 2},
    # J3, J4 (elbow) - mid joints
    {"T_coulomb": 0.25, "T_static": 0.35, "omega_s": 0.05, "delta": 2},
    {"T_coulomb": 0.25, "T_static": 0.35, "omega_s": 0.05, "delta": 2},
    # J5, J6, J7 (wrist) - smaller joints
    {"T_coulomb": 0.15, "T_static": 0.2, "omega_s": 0.05, "delta": 2},
    {"T_coulomb": 0.15, "T_static": 0.2, "omega_s": 0.05, "delta": 2},
    {"T_coulomb": 0.15, "T_static": 0.2, "omega_s": 0.05, "delta": 2},
]

motors = [
    Motor(position_limit=2*np.pi, velocity_limit=10.0, torque_limit=50.0,
          use_mit_controller=False, **params)
    for params in friction_params
]

# Set initial joint positions (modify these to change starting pose)
initial_qpos = [0, 0, 0, 0, 0, 0, 0]
data.qpos[:7] = initial_qpos
mujoco.mj_forward(model, data)

print("=" * 50)
print("Kinova Joint Viewer (with Motor class + 2ms delay)")
print("=" * 50)
print("Drag the arm in the viewer to move joints.")
print("Joint positions are printed continuously.")
print("Torque commands pass through Motor objects with 2ms delay.")
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
    """Compute gravity compensation torques and send through motor objects."""
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
    
    # Send torque commands through motor objects
    for i, motor in enumerate(motors):
        motor.set_torque_command(gravity_torques[i])
        motor.update(data.qpos[i], data.qvel[i])
        data.ctrl[i] = motor.get_output_torque()

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
