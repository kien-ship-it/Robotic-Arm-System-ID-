"""
Play the test trajectory from the validation.
Run with: mjpython play_test_trajectory.py
"""
import mujoco
import mujoco.viewer
import numpy as np
import os
import time


def generate_smooth_trajectory(duration=10.0, hz=1000, seed=123):
    """Same trajectory generator as validation - seed=123 for test."""
    np.random.seed(seed)
    n_samples = int(duration * hz)
    t = np.linspace(0, duration, n_samples)
    
    trajectory = np.zeros((n_samples, 7))
    velocity = np.zeros((n_samples, 7))
    
    for j in range(7):
        n_sines = np.random.randint(3, 5)
        for _ in range(n_sines):
            freq = np.random.uniform(0.1, 0.6)
            amp = np.random.uniform(0.15, 0.35)
            phase = np.random.uniform(0, 2*np.pi)
            omega = 2 * np.pi * freq
            
            trajectory[:, j] += amp * np.sin(omega * t + phase)
            velocity[:, j] += amp * omega * np.cos(omega * t + phase)
    
    return trajectory, velocity, t


def main():
    model_path = os.path.join(os.path.dirname(__file__), "model", "kinova_fullinertia.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    print("Generating test trajectory (seed=123)...")
    trajectory, velocity, t_array = generate_smooth_trajectory(duration=10.0, seed=123)
    
    frame_idx = 0
    n_frames = len(t_array)
    dt = 0.001
    
    print(f"Playing {n_frames} frames ({n_frames*dt:.1f} seconds)")
    print("Close viewer to exit")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            if frame_idx >= n_frames:
                frame_idx = 0
            
            data.qpos[:7] = trajectory[frame_idx, :]
            data.qvel[:7] = velocity[frame_idx, :]
            mujoco.mj_forward(model, data)
            
            frame_idx += 1
            time.sleep(dt)
            viewer.sync()


if __name__ == "__main__":
    main()
