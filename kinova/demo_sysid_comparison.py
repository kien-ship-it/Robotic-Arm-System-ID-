"""
Demo: Side-by-side comparison of MuJoCo inverse dynamics vs Regressor torque prediction.

Creates a video with:
- Left: MuJoCo viewer rendering
- Right: Two torque plots (MuJoCo vs Regressor) advancing with time

Run with: mjpython demo_sysid_comparison.py
"""
import mujoco
import numpy as np
import os
import sys

# Add parent for imports
sys.path.insert(0, os.path.dirname(__file__))
from generateRegressor import compute_joint_regressor_row


def generate_6dof_excitation(duration=10.0, hz=1000):
    """Generate smooth 6-DOF excitation trajectory."""
    n_samples = int(duration * hz)
    t = np.linspace(0, duration, n_samples)
    
    trajectory = np.zeros((n_samples, 7))
    velocity = np.zeros((n_samples, 7))
    acceleration = np.zeros((n_samples, 7))
    
    np.random.seed(42)
    
    for j in range(7):
        n_sines = np.random.randint(3, 6)
        for _ in range(n_sines):
            freq = np.random.uniform(0.1, 0.8)
            amp = np.random.uniform(0.2, 0.5)
            phase = np.random.uniform(0, 2*np.pi)
            omega = 2 * np.pi * freq
            
            trajectory[:, j] += amp * np.sin(omega * t + phase)
            velocity[:, j] += amp * omega * np.cos(omega * t + phase)
            acceleration[:, j] -= amp * omega**2 * np.sin(omega * t + phase)
    
    return trajectory, velocity, acceleration, t


def main():
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    import cv2
    
    # Load model
    model_path = os.path.join(os.path.dirname(__file__), "model", "kinova_fullinertia.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Get IDs
    link6_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
    joint6_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "joint6")
    joint6_axis = np.array([0, -1, 0])
    
    # Jacobians
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    g_world = np.array([0.0, 0.0, 9.81])
    
    # Ground truth parameters
    theta_true = np.array([
        0.789160, -0.000047, 0.009093, 0.120437,
        0.024382, 0.000001, 0.000009, 0.024312, -0.001638, 0.000551
    ])
    
    # Generate trajectory
    print("Generating trajectory...")
    duration = 10.0
    trajectory, velocity, acceleration, t_array = generate_6dof_excitation(duration=duration)
    n_frames = len(t_array)
    
    # Pre-compute all torques
    print("Pre-computing torques...")
    tau_mujoco = np.zeros(n_frames)
    tau_regressor = np.zeros(n_frames)
    
    for i in range(n_frames):
        data.qpos[:7] = trajectory[i, :]
        data.qvel[:7] = velocity[i, :]
        data.qacc[:7] = acceleration[i, :]
        
        mujoco.mj_forward(model, data)
        
        # MuJoCo inverse dynamics
        mujoco.mj_inverse(model, data)
        tau_mujoco[i] = data.qfrc_inverse[5]
        
        # Regressor prediction
        mujoco.mj_jacSite(model, data, jacp, jacr, joint6_site_id)
        v_world = jacp @ data.qvel
        w_world = jacr @ data.qvel
        
        R = data.xmat[link6_body_id].reshape(3, 3)
        w_local = R.T @ w_world
        v_local = R.T @ v_world
        
        spatial_acc_world = np.zeros(6)
        mujoco.mj_objectAcceleration(model, data, mujoco.mjtObj.mjOBJ_SITE, joint6_site_id, spatial_acc_world, 0)
        dv_world = spatial_acc_world[:3]
        dw_world = spatial_acc_world[3:]
        
        dw_local = R.T @ dw_world
        dv_local = R.T @ (dv_world + g_world) - np.cross(w_local, v_local)
        
        Y_row = compute_joint_regressor_row(
            w_local[0], w_local[1], w_local[2],
            v_local[0], v_local[1], v_local[2],
            dw_local[0], dw_local[1], dw_local[2],
            dv_local[0], dv_local[1], dv_local[2],
            joint6_axis
        )
        tau_regressor[i] = Y_row @ theta_true
        
        if i % 1000 == 0:
            print(f"  {i}/{n_frames}")
    
    # Compute error
    rmse = np.sqrt(np.mean((tau_mujoco - tau_regressor)**2))
    print(f"RMSE: {rmse:.6f} Nm")
    
    # Setup rendering
    print("Rendering video...")
    render_width, render_height = 640, 480
    renderer = mujoco.Renderer(model, height=render_height, width=render_width)
    
    # Video settings
    fps = 30
    frame_skip = int(1000 / fps)  # 1000 Hz trajectory -> 30 fps video
    video_frames = n_frames // frame_skip
    
    # Output video
    output_path = os.path.join(os.path.dirname(__file__), "sysid_comparison.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    
    # Combined frame size: robot (640) + plots (640) = 1280 x 480
    combined_width = render_width + 640
    combined_height = render_height
    video = cv2.VideoWriter(output_path, fourcc, fps, (combined_width, combined_height))
    
    # Create figure for plots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6.4, 4.8), dpi=100)
    fig.tight_layout(pad=2.0)
    
    # Plot styling
    window_sec = 3.0  # Show 3 seconds of data
    window_samples = int(window_sec * 1000)
    
    for frame_i in range(video_frames):
        traj_idx = frame_i * frame_skip
        current_time = t_array[traj_idx]
        
        # Set robot state
        data.qpos[:7] = trajectory[traj_idx, :]
        data.qvel[:7] = velocity[traj_idx, :]
        mujoco.mj_forward(model, data)
        
        # Render robot
        renderer.update_scene(data)
        robot_frame = renderer.render()
        robot_frame_bgr = cv2.cvtColor(robot_frame, cv2.COLOR_RGB2BGR)
        
        # Create plot frame
        ax1.clear()
        ax2.clear()
        
        # Determine plot window
        start_idx = max(0, traj_idx - window_samples)
        end_idx = min(n_frames, traj_idx + window_samples // 3)
        
        t_window = t_array[start_idx:end_idx]
        tau_mj_window = tau_mujoco[start_idx:end_idx]
        tau_reg_window = tau_regressor[start_idx:end_idx]
        
        # Plot MuJoCo torque
        ax1.plot(t_window, tau_mj_window, 'b-', linewidth=1.5, label='MuJoCo Inverse')
        ax1.axvline(x=current_time, color='r', linestyle='--', alpha=0.7)
        ax1.set_ylabel('Torque (Nm)')
        ax1.set_title('MuJoCo Inverse Dynamics', fontsize=10)
        ax1.legend(loc='upper right', fontsize=8)
        ax1.set_xlim(t_window[0], t_window[-1])
        ax1.grid(True, alpha=0.3)
        
        # Plot Regressor torque
        ax2.plot(t_window, tau_reg_window, 'g-', linewidth=1.5, label='Regressor')
        ax2.axvline(x=current_time, color='r', linestyle='--', alpha=0.7)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Torque (Nm)')
        ax2.set_title(f'Regressor Prediction (RMSE: {rmse:.4f} Nm)', fontsize=10)
        ax2.legend(loc='upper right', fontsize=8)
        ax2.set_xlim(t_window[0], t_window[-1])
        ax2.grid(True, alpha=0.3)
        
        # Match y-axis limits
        y_min = min(tau_mj_window.min(), tau_reg_window.min()) - 0.5
        y_max = max(tau_mj_window.max(), tau_reg_window.max()) + 0.5
        ax1.set_ylim(y_min, y_max)
        ax2.set_ylim(y_min, y_max)
        
        # Render plot to image
        fig.canvas.draw()
        plot_frame = np.array(fig.canvas.buffer_rgba())[:, :, :3]  # RGBA -> RGB
        plot_frame_bgr = cv2.cvtColor(plot_frame, cv2.COLOR_RGB2BGR)
        
        # Combine frames
        combined = np.zeros((combined_height, combined_width, 3), dtype=np.uint8)
        combined[:, :render_width, :] = robot_frame_bgr
        combined[:, render_width:, :] = plot_frame_bgr
        
        video.write(combined)
        
        if frame_i % 30 == 0:
            print(f"  Frame {frame_i}/{video_frames} ({100*frame_i/video_frames:.0f}%)")
    
    video.release()
    plt.close(fig)
    
    print(f"\nSaved video to: {output_path}")
    print(f"Duration: {duration:.1f}s, {video_frames} frames at {fps} fps")


if __name__ == "__main__":
    main()
