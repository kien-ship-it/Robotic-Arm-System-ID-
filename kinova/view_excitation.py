"""
Visualize the 6-DOF excitation trajectory used for system ID verification.
Run with: mjpython view_excitation.py
"""
import mujoco
import mujoco.viewer
import numpy as np
import os
import time


def generate_6dof_excitation(duration=10.0, hz=1000):
    """
    Generate random 6-DOF excitation similar to test_mujoco_inverse_fit.
    Returns smooth trajectory by interpolating between random waypoints.
    """
    n_samples = int(duration * hz)
    t = np.linspace(0, duration, n_samples)
    
    # Use sum of sinusoids at different frequencies for each joint
    # This creates smooth, continuous motion
    trajectory = np.zeros((n_samples, 7))
    velocity = np.zeros((n_samples, 7))
    
    np.random.seed(42)
    
    # Each joint gets 3-5 sinusoids with random phases
    for j in range(7):
        n_sines = np.random.randint(3, 6)
        for _ in range(n_sines):
            freq = np.random.uniform(0.1, 0.8)  # Hz
            amp = np.random.uniform(0.2, 0.5)   # rad
            phase = np.random.uniform(0, 2*np.pi)
            omega = 2 * np.pi * freq
            
            trajectory[:, j] += amp * np.sin(omega * t + phase)
            velocity[:, j] += amp * omega * np.cos(omega * t + phase)
    
    return trajectory, velocity


def main():
    model_path = os.path.join(os.path.dirname(__file__), "model", "kinova_fullinertia.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Generate excitation trajectory
    print("Generating 6-DOF excitation trajectory...")
    trajectory, velocity = generate_6dof_excitation(duration=15.0)
    
    frame_idx = 0
    num_frames = trajectory.shape[0]
    dt = 0.001
    
    print(f"Playing {num_frames} frames ({num_frames*dt:.1f} seconds)")
    print("Close viewer window to exit")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            if frame_idx >= num_frames:
                frame_idx = 0  # Loop
            
            # Set joint positions from trajectory
            data.qpos[:7] = trajectory[frame_idx, :]
            data.qvel[:7] = velocity[frame_idx, :]
            
            # Forward kinematics only
            mujoco.mj_forward(model, data)
            
            frame_idx += 1
            time.sleep(dt)
            viewer.sync()


if __name__ == "__main__":
    main()
