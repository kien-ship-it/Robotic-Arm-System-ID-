"""
Create overlay video: MuJoCo rendering + torque comparison plots.

Uses the validated pipeline:
1. Train on trajectory with seed=42
2. Show test trajectory with seed=123
3. Compare regressor prediction vs MuJoCo inverse dynamics

Run with: mjpython create_overlay_video.py
"""
import mujoco
import numpy as np
import os
import sys
import cv2

sys.path.insert(0, os.path.dirname(__file__))
from generateRegressor import compute_joint_regressor_row


def generate_smooth_trajectory(duration=10.0, hz=1000, seed=42):
    """Generate smooth multi-frequency trajectory."""
    np.random.seed(seed)
    n_samples = int(duration * hz)
    t = np.linspace(0, duration, n_samples)
    
    trajectory = np.zeros((n_samples, 7))
    velocity = np.zeros((n_samples, 7))
    acceleration = np.zeros((n_samples, 7))
    
    for j in range(7):
        n_sines = np.random.randint(3, 5)
        for _ in range(n_sines):
            freq = np.random.uniform(0.1, 0.6)
            amp = np.random.uniform(0.15, 0.35)
            phase = np.random.uniform(0, 2*np.pi)
            omega = 2 * np.pi * freq
            
            trajectory[:, j] += amp * np.sin(omega * t + phase)
            velocity[:, j] += amp * omega * np.cos(omega * t + phase)
            acceleration[:, j] -= amp * omega**2 * np.sin(omega * t + phase)
    
    return trajectory, velocity, acceleration, t


def collect_data(model, data, trajectory, velocity, acceleration):
    """Collect regressor and MuJoCo inverse dynamics data."""
    link6_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
    joint6_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "joint6")
    joint6_axis = np.array([0, -1, 0])
    
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    g_world = np.array([0.0, 0.0, 9.81])
    
    n_samples = len(trajectory)
    Y_list = []
    tau_mj_list = []
    eps = 1e-6
    
    for i in range(n_samples):
        qpos = trajectory[i, :]
        qvel = velocity[i, :]
        qacc = acceleration[i, :]
        
        data.qpos[:7] = qpos
        data.qvel[:7] = qvel
        data.qacc[:7] = qacc
        
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_crb(model, data)
        
        mujoco.mj_jacSite(model, data, jacp, jacr, joint6_site_id)
        v_world = jacp @ qvel
        w_world = jacr @ qvel
        
        R = data.xmat[link6_body_id].reshape(3, 3)
        w_local = R.T @ w_world
        v_local = R.T @ v_world
        
        # Jdot @ qvel
        data.qpos[:7] = qpos + eps * qvel
        mujoco.mj_kinematics(model, data)
        jacp_plus = np.zeros((3, model.nv))
        jacr_plus = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp_plus, jacr_plus, joint6_site_id)
        
        Jdot_v_linear = (jacp_plus - jacp) @ qvel / eps
        Jdot_v_angular = (jacr_plus - jacr) @ qvel / eps
        
        dv_world = jacp @ qacc + Jdot_v_linear
        dw_world = jacr @ qacc + Jdot_v_angular
        
        dv_world_with_g = dv_world + g_world
        dw_local = R.T @ dw_world
        dv_local = R.T @ dv_world_with_g - np.cross(w_local, v_local)
        
        Y_row = compute_joint_regressor_row(
            w_local[0], w_local[1], w_local[2],
            v_local[0], v_local[1], v_local[2],
            dw_local[0], dw_local[1], dw_local[2],
            dv_local[0], dv_local[1], dv_local[2],
            joint6_axis
        )
        
        # Restore and get MuJoCo inverse
        data.qpos[:7] = qpos
        data.qvel[:7] = qvel
        data.qacc[:7] = qacc
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_crb(model, data)
        mujoco.mj_inverse(model, data)
        tau_mj = data.qfrc_inverse[5]
        
        Y_list.append(Y_row)
        tau_mj_list.append(tau_mj)
    
    return np.array(Y_list), np.array(tau_mj_list)


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    
    print("=" * 60)
    print("Creating Overlay Video")
    print("=" * 60)
    
    model_path = os.path.join(os.path.dirname(__file__), "model", "kinova_fullinertia.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Step 1: Train on seed=42
    print("\n[1] Training on trajectory (seed=42)...")
    train_traj, train_vel, train_acc, train_t = generate_smooth_trajectory(duration=15.0, seed=42)
    Y_train, tau_train = collect_data(model, data, train_traj, train_vel, train_acc)
    
    theta_fit, _, rank, _ = np.linalg.lstsq(Y_train, tau_train, rcond=1e-10)
    train_rmse = np.sqrt(np.mean((tau_train - Y_train @ theta_fit)**2))
    print(f"  Training RMSE: {train_rmse:.4f} Nm")
    
    # Step 2: Test trajectory (seed=123)
    print("\n[2] Generating test trajectory (seed=123)...")
    test_traj, test_vel, test_acc, test_t = generate_smooth_trajectory(duration=10.0, seed=123)
    Y_test, tau_mujoco = collect_data(model, data, test_traj, test_vel, test_acc)
    tau_regressor = Y_test @ theta_fit
    
    test_rmse = np.sqrt(np.mean((tau_mujoco - tau_regressor)**2))
    print(f"  Test RMSE: {test_rmse:.4f} Nm")
    
    # Step 3: Render video
    print("\n[3] Rendering video...")
    render_width, render_height = 640, 480
    renderer = mujoco.Renderer(model, height=render_height, width=render_width)
    
    fps = 30
    frame_skip = int(1000 / fps)
    n_video_frames = len(test_t) // frame_skip
    
    output_path = os.path.join(os.path.dirname(__file__), "sysid_overlay.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    combined_width = render_width + 640
    video = cv2.VideoWriter(output_path, fourcc, fps, (combined_width, render_height))
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6.4, 4.8), dpi=100)
    fig.tight_layout(pad=2.0)
    
    window_sec = 3.0
    window_samples = int(window_sec * 1000)
    
    for frame_i in range(n_video_frames):
        traj_idx = frame_i * frame_skip
        current_time = test_t[traj_idx]
        
        # Robot render
        data.qpos[:7] = test_traj[traj_idx, :]
        data.qvel[:7] = test_vel[traj_idx, :]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data)
        robot_frame = renderer.render()
        robot_bgr = cv2.cvtColor(robot_frame, cv2.COLOR_RGB2BGR)
        
        # Plot
        ax1.clear()
        ax2.clear()
        
        start_idx = max(0, traj_idx - window_samples)
        end_idx = min(len(test_t), traj_idx + window_samples // 3)
        
        t_win = test_t[start_idx:end_idx]
        tau_mj_win = tau_mujoco[start_idx:end_idx]
        tau_reg_win = tau_regressor[start_idx:end_idx]
        
        ax1.plot(t_win, tau_mj_win, 'b-', lw=1.5, label='MuJoCo Inverse')
        ax1.axvline(x=current_time, color='r', ls='--', alpha=0.7)
        ax1.set_ylabel('Torque (Nm)')
        ax1.set_title('MuJoCo Inverse Dynamics (Ground Truth)', fontsize=10)
        ax1.legend(loc='upper right', fontsize=8)
        ax1.set_xlim(t_win[0], t_win[-1])
        ax1.grid(True, alpha=0.3)
        
        ax2.plot(t_win, tau_reg_win, 'g-', lw=1.5, label='Regressor')
        ax2.axvline(x=current_time, color='r', ls='--', alpha=0.7)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Torque (Nm)')
        ax2.set_title(f'Regressor Prediction (RMSE: {test_rmse:.4f} Nm)', fontsize=10)
        ax2.legend(loc='upper right', fontsize=8)
        ax2.set_xlim(t_win[0], t_win[-1])
        ax2.grid(True, alpha=0.3)
        
        y_min = min(tau_mj_win.min(), tau_reg_win.min()) - 0.2
        y_max = max(tau_mj_win.max(), tau_reg_win.max()) + 0.2
        ax1.set_ylim(y_min, y_max)
        ax2.set_ylim(y_min, y_max)
        
        fig.canvas.draw()
        plot_frame = np.array(fig.canvas.buffer_rgba())[:, :, :3]
        plot_bgr = cv2.cvtColor(plot_frame, cv2.COLOR_RGB2BGR)
        
        combined = np.zeros((render_height, combined_width, 3), dtype=np.uint8)
        combined[:, :render_width, :] = robot_bgr
        combined[:, render_width:, :] = plot_bgr
        
        video.write(combined)
        
        if frame_i % 30 == 0:
            print(f"  Frame {frame_i}/{n_video_frames} ({100*frame_i/n_video_frames:.0f}%)")
    
    video.release()
    plt.close(fig)
    
    print(f"\n✓ Saved: {output_path}")
    print(f"  Duration: {test_t[-1]:.1f}s, {n_video_frames} frames @ {fps} fps")
    print(f"  Test RMSE: {test_rmse:.4f} Nm")


if __name__ == "__main__":
    main()
