"""
Real-time Cartesian velocity viewer for each joint.
Displays linear velocity (vx, vy, vz) at each joint body in the MuJoCo viewer.

Run with mjpython:
    mjpython velocity_viewer.py
"""
import mujoco
import mujoco.viewer
import numpy as np
import os


def view_trajectory():
    """
    Simply play the trajectory on the arm using mj_forward (no dynamics/control).
    Useful for visualizing the desired motion.
    """
    model_path = os.path.join(os.path.dirname(__file__), "model", "kinova_fullinertia.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    
    # Generate demo trajectory
    TEST_JOINT = None  # Change to 0-6 to test single joint, or None for all
    trajectory, velocity = generate_demo_trajectory(seconds=10.0, test_joint=TEST_JOINT)
    
    frame_idx = 0
    num_frames = trajectory.shape[0]
    dt = 0.001
    
    import time
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            if frame_idx >= num_frames:
                frame_idx = 0
            
            # Directly set joint positions from trajectory
            data.qpos[:7] = trajectory[frame_idx, :]
            data.qvel[:7] = velocity[frame_idx, :]
            
            # Forward kinematics only (no dynamics)
            mujoco.mj_forward(model, data)
            
            frame_idx += 1
            time.sleep(dt)
            viewer.sync()


def system_id():
    """
    System identification function for the Kinova arm.
    Uses the guess model for parameter estimation.
    """
    from motor import Motor
    
    model_path = os.path.join(os.path.dirname(__file__), "model", "kinova_fullinertia.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    model_path_guess = os.path.join(os.path.dirname(__file__), "model", "kinova_fullinertia_guess.xml")
    model_guess = mujoco.MjModel.from_xml_path(model_path_guess)
    data_guess = mujoco.MjData(model_guess)
    # Create 7 motor objects with MIT controller
    # PD gains estimated for Kinova arm:
    # - Joints 1-4 (shoulder/elbow): higher gains for larger joints
    # - Joints 5-7 (wrist): lower gains for smaller joints
    # 
    # Friction parameters (Stribeck model) estimated for harmonic drive actuators:
    # - T_coulomb: Coulomb friction ~1-3% of max torque for larger joints, ~2-4% for wrist
    # - T_static: Static friction ~1.2-1.5x Coulomb friction
    # - omega_s: Stribeck velocity ~0.1-0.3 rad/s typical for harmonic drives
    motors = []
    
    # Joint parameters: (position_limit, velocity_limit, torque_limit, kp, kd, T_coulomb, T_static, omega_s)
    # kd=0 for now, kp tuned for ~500 rad/s bandwidth based on inertia at home position
    # kp = omega_n^2 * I, where omega_n = 500 rad/s
    # Inertias at home: J0=0.0036, J1=0.91, J2=0.0023, J3=0.20, J4=0.0014, J5=0.024, J6=0.0011
    joint_params = [
        (np.pi, 2.0, 1000.0, 912.0, 0.0, None, None, None),     # Joint 0: 500^2 * 0.0036
        (np.pi, 2.0, 1000.0, 200.0, 0.0, None, None, None),  # Joint 1: 500^2 * 0.91
        (np.pi, 2.0, 1000.0, 583.0, 0.0, None, None, None),     # Joint 2: 500^2 * 0.0023
        (np.pi, 2.0, 1000.0, 500.0, 0.0, None, None, None),   # Joint 3: 500^2 * 0.20
        (np.pi, 2.5, 1000.0, 350.0, 0.0, None, None, None),     # Joint 4: 500^2 * 0.0014
        (np.pi, 2.5, 1000.0, 500.0, 0.0, None, None, None),    # Joint 5: 500^2 * 0.024
        (np.pi, 2.5, 1000.0, 271.0, 0.0, None, None, None),     # Joint 6: 500^2 * 0.0011
    ]
    
    for i, (pos_lim, vel_lim, torque_lim, kp, kd, T_c, T_s, omega_s) in enumerate(joint_params):
        motor = Motor(
            position_limit=pos_lim,
            velocity_limit=vel_lim,
            torque_limit=torque_lim,
            use_mit_controller=True,
            T_coulomb=T_c,
            T_static=T_s,
            omega_s=omega_s
        )
        motor.set_mit_params(kp=kp, kd=kd)
        motors.append(motor)
    
    # Generate demo trajectory - test one joint at a time (0-6), or None for all
    TEST_JOINT = None  # Change this to test different joints: 0-6, or None for all
    trajectory, velocity = generate_demo_trajectory(seconds=10.0, test_joint=TEST_JOINT)
    
    # Initialize simulation at first trajectory position
    data.qpos[:7] = trajectory[0, :]
    data.qvel[:7] = velocity[0, :]
    mujoco.mj_forward(model, data)
    
    # Initialize motors with current state
    for i, motor in enumerate(motors):
        motor.update(data.qpos[i], data.qvel[i], initialize=True)
    
    # Replay motion with feedforward + feedback control
    frame_idx = 0
    num_frames = trajectory.shape[0]
    
    dt = model.opt.timestep  # Use model timestep
    slowdown = 1  # Real-time playback
    
    # Logging arrays for first pass through trajectory
    # Cartesian velocities: 7 joints x 3 components (vx, vy, vz) in local frame
    log_cartesian_vel = np.zeros((num_frames, 7, 3))
    log_time = np.zeros(num_frames)
    first_pass = True
    
    import time
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            if frame_idx >= num_frames:
                # Loop trajectory
                first_pass = False  # Done logging
                frame_idx = 0
                data.qpos[:7] = trajectory[0, :]
                data.qvel[:7] = velocity[0, :]
                mujoco.mj_forward(model, data)
                for i, motor in enumerate(motors):
                    motor.reset()
                    motor.update(data.qpos[i], data.qvel[i], initialize=True)
            
            # Step 1: Calculate feedforward torques
            # Simple approach: gravity compensation at actual state only
            # The PD controller will handle trajectory tracking
            
            data_guess.qpos[:7] = data.qpos[:7].copy()
            data_guess.qvel[:7] = 0  # Zero velocity for pure gravity compensation
            data_guess.qacc[:7] = 0  # Zero acceleration
            mujoco.mj_inverse(model_guess, data_guess)
            feedforward_torques = data_guess.qfrc_inverse[:7].copy()
            
            # Step 2: For each motor, set feedforward torque and desired position from trajectory
            for i, motor in enumerate(motors):
                # For non-test joints, hold at zero with high gains
                if TEST_JOINT is not None and i != TEST_JOINT:
                    motor.set_mit_params(
                        kp=500.0,  # High stiffness to lock joint
                        kd=0.0,    # No velocity term (encoder noise)
                        desired_pos=0.0,
                        desired_vel=0.0,
                        ff_torque=feedforward_torques[i]
                    )
                else:
                    motor.set_mit_params(
                        desired_pos=trajectory[frame_idx, i],
                        desired_vel=velocity[frame_idx, i],
                        ff_torque=feedforward_torques[i]
                    )
            #Step 3: update motor states with current simulation state
            for i, motor in enumerate(motors):
                motor.update(data.qpos[i], data.qvel[i])
            
            # Log Cartesian velocities on first pass
            if first_pass and frame_idx < num_frames:
                log_time[frame_idx] = frame_idx * dt
                cart_vels = get_cartesian_velocities(model, data)
                for i, v in enumerate(cart_vels):
                    log_cartesian_vel[frame_idx, i, :] = v['linear_local']
        
            # Step 4: Compute motor output torques (includes PD feedback + feedforward)
            output_torques = np.zeros(7)
            for i, motor in enumerate(motors):
                output_torques[i] = motor.get_output_torque()
            
            # Step 5: Apply torques to simulation and step
            data.ctrl[:7] = output_torques
            mujoco.mj_step(model, data)            
            
            frame_idx += 1
            
            # Slow down playback
            time.sleep(dt * slowdown)
            
            # Print Cartesian velocities (overwrite same lines using ANSI escape codes)
            if frame_idx % 100 == 0:
                cart_vels = get_cartesian_velocities(model, data)
                # Build output as single string
                lines = [f"Frame {frame_idx:5d}"]
                lines.append(f"{'Joint':<8} {'vx':>10} {'vy':>10} {'vz':>10}")
                for i, v in enumerate(cart_vels):
                    lv = v['linear_local']
                    lines.append(f"Joint {i+1:<3} {lv[0]:>+10.4f} {lv[1]:>+10.4f} {lv[2]:>+10.4f}")
                # Clear screen and move cursor to top-left, then print
                print("\033[H\033[J" + "\n".join(lines), flush=True)
            
            viewer.sync()
    
    # Save logged data to CSV
    log_file = os.path.join(os.path.dirname(__file__), "cartesian_velocity_log.csv")
    header = "time," + ",".join([f"joint{i+1}_vx,joint{i+1}_vy,joint{i+1}_vz" for i in range(7)])
    log_data = np.zeros((num_frames, 1 + 7*3))
    log_data[:, 0] = log_time
    for i in range(7):
        log_data[:, 1 + i*3] = log_cartesian_vel[:, i, 0]  # vx
        log_data[:, 2 + i*3] = log_cartesian_vel[:, i, 1]  # vy
        log_data[:, 3 + i*3] = log_cartesian_vel[:, i, 2]  # vz
    np.savetxt(log_file, log_data, delimiter=",", header=header, comments="")
    print(f"\nSaved Cartesian velocity log to {log_file}")
    
    # Plot Cartesian velocities (use Agg backend to avoid macOS threading issues with mjpython)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    
    fig, axes = plt.subplots(4, 2, figsize=(12, 10))
    axes = axes.flatten()
    
    for i in range(7):
        ax = axes[i]
        ax.plot(log_time, log_cartesian_vel[:, i, 0], 'r-', label='vx', alpha=0.7)
        ax.plot(log_time, log_cartesian_vel[:, i, 1], 'g-', label='vy', alpha=0.7)
        ax.plot(log_time, log_cartesian_vel[:, i, 2], 'b-', label='vz', alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title(f'Joint {i+1} Cartesian Velocity (local frame)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    
    # Hide the 8th subplot
    axes[7].set_visible(False)
    
    plt.tight_layout()
    plot_file = os.path.join(os.path.dirname(__file__), "cartesian_velocity_plot.png")
    plt.savefig(plot_file, dpi=150)
    plt.close()  # Close without showing (avoids macOS threading issue with mjpython)
    print(f"Saved Cartesian velocity plot to {plot_file}")

def get_cartesian_velocities(model, data):
    """Compute Cartesian velocity at each joint site in local body frame."""
    velocities = []
    
    # Build joint_info from model - sites named joint1-joint7, bodies named link1-link7
    for i in range(1, 8):
        site_name = f"joint{i}"
        body_name = f"link{i}"
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        
        # Allocate Jacobians
        jacp = np.zeros((3, model.nv))  # translational
        jacr = np.zeros((3, model.nv))  # rotational
        
        # Compute Jacobian for this site (at joint origin)
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
        
        # Cartesian velocity in world frame = J @ qvel
        linear_vel_world = jacp @ data.qvel
        angular_vel_world = jacr @ data.qvel
        
        # Get body rotation matrix (3x3) from data.xmat (stored as 9-element flat array)
        R_body = data.xmat[body_id].reshape(3, 3)
        
        # Transform to local body frame: v_local = R^T @ v_world
        linear_vel_local = R_body.T @ linear_vel_world
        angular_vel_local = R_body.T @ angular_vel_world
        
        velocities.append({
            'name': site_name,
            'site_id': site_id,
            'body_id': body_id,
            'linear_world': linear_vel_world.copy(),
            'linear_local': linear_vel_local.copy(),
            'angular_world': angular_vel_world.copy(),
            'angular_local': angular_vel_local.copy(),
            'speed': np.linalg.norm(linear_vel_local)
        })
    
    return velocities

def format_velocity_display(velocities):
    """Format velocities for display."""
    lines = ["Cartesian Velocities in Local Frame (m/s):", ""]
    for v in velocities:
        lv = v['linear_local']
        lines.append(f"{v['name']:>8}: [{lv[0]:+6.3f}, {lv[1]:+6.3f}, {lv[2]:+6.3f}]  |v|={v['speed']:.3f}")
    return "\n".join(lines)

# Generate a simple oscillating trajectory for demo
def generate_demo_trajectory(seconds: float, hz: int = 1000, test_joint: int = None):
    """Generate oscillating motion on joints.
    
    Args:
        seconds: Duration of trajectory
        hz: Sample rate
        test_joint: If specified (0-6), only move this joint. If None, move all.
    
    Returns:
        trajectory: (num_samples, 7) array of joint positions
        velocity: (num_samples, 7) array of joint velocities
    """
    num_samples = int(seconds * hz)
    t = np.linspace(0, seconds, num_samples)
    trajectory = np.zeros((num_samples, 7))
    velocity = np.zeros((num_samples, 7))
    
    # Different frequencies for each joint
    freqs = [0.31, 0.37, 0.43, 0.53, 0.59, 0, 0]
    amps = [0.5, 0.4, 0.3, 0.3, 0.2, 0.2, 0.2]
    
    if test_joint is not None:
        # Only move the specified joint
        joints_to_move = [test_joint]
        print(f"Testing joint {test_joint} only")
    else:
        # Move first 4 joints
        joints_to_move = range(4)
    
    for j in joints_to_move:
        omega = 2 * np.pi * freqs[j]
        # Use (1 - cos) so trajectory starts at position=0 with velocity=0
        trajectory[:, j] = amps[j] * np.cos(omega * t)
        velocity[:, j] = -amps[j] * omega * np.sin(omega * t)  # d/dt[cos(ωt)] = -ω*sin(ωt)
    
    return trajectory, velocity

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "view":
        view_trajectory()
    else:
        system_id()
