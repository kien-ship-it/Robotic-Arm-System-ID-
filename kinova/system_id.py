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


def view_system_id_trajectory():
    """
    View the system ID trajectory without dynamics - just forward kinematics.
    Use this to check the trajectory looks reasonable before running full sim.
    """
    model_path = os.path.join(os.path.dirname(__file__), "model", "kinova_fullinertia.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Generate system ID trajectory
    trajectory, velocity = generate_demo_trajectory(seconds=15.0, for_system_id=True)
    
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
    Collects spatial velocity, acceleration, and torque data for link6 (composite with link7).
    
    ****************************************************************************
    * NOTE: Currently using MuJoCo ground truth for velocities/accelerations.  *
    * TODO: Change to encoder values when moving to real hardware.             *
    * When using encoder data, add filtering to acceleration (finite diff      *
    * will be noisy). Consider Butterworth low-pass filter.                    *
    ****************************************************************************
    """
    from motor import Motor
    
    model_path = os.path.join(os.path.dirname(__file__), "model", "kinova_fullinertia.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    model_path_guess = os.path.join(os.path.dirname(__file__), "model", "kinova_fullinertia_guess.xml")
    model_guess = mujoco.MjModel.from_xml_path(model_path_guess)
    data_guess = mujoco.MjData(model_guess)
    
    # Create 7 motor objects with MIT controller
    motors = []
    
    # Joint parameters: (position_limit, velocity_limit, torque_limit, kp, kd, T_coulomb, T_static, omega_s)
    # kd=0 for now, kp tuned for ~500 rad/s bandwidth based on inertia at home position
    # kp = omega_n^2 * I, where omega_n = 500 rad/s
    # Inertias at home: J0=0.0036, J1=0.91, J2=0.0023, J3=0.20, J4=0.0014, J5=0.024, J6=0.0011
    joint_params = [
        (np.pi, 2.0, 1000.0, 912.0, 0.0, None, None, None),     # Joint 0
        (np.pi, 2.0, 1000.0, 200.0, 0.0, None, None, None),     # Joint 1
        (np.pi, 2.0, 1000.0, 583.0, 0.0, None, None, None),     # Joint 2
        (np.pi, 2.0, 1000.0, 500.0, 0.0, None, None, None),     # Joint 3
        (np.pi, 2.5, 1000.0, 350.0, 0.0, None, None, None),     # Joint 4
        (np.pi, 2.5, 1000.0, 500.0, 0.0, None, None, None),     # Joint 5 (joint6)
        (np.pi, 2.5, 1000.0, 271.0, 0.0, None, None, None),     # Joint 6 (joint7)
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
    
    # Generate trajectory with multi-frequency excitation for system ID
    # Joint 6 (index 5) moves, joint 7 (index 6) stays at 0
    trajectory, velocity = generate_demo_trajectory(seconds=15.0, for_system_id=True)
    
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
    
    dt = model.opt.timestep  # Use model timestep (0.001s)
    slowdown = 1  # Real-time playback
    
    # Logging arrays for system ID data (link6)
    # Spatial velocity: [wx, wy, wz, vx, vy, vz] in link6 frame
    # Spatial acceleration: [dwx, dwy, dwz, dvx, dvy, dvz] in link6 frame (with gravity comp)
    # Torque: tau6 (commanded torque for joint6)
    log_spatial_vel = np.zeros((num_frames, 6))      # [wx, wy, wz, vx, vy, vz]
    log_spatial_acc = np.zeros((num_frames, 6))      # [dwx, dwy, dwz, dvx, dvy, dvz]
    log_torque = np.zeros(num_frames)                # tau6
    log_time = np.zeros(num_frames)
    
    # Previous velocity for finite difference
    prev_spatial_vel = None
    
    # Gravity in world frame
    g_world = np.array([0.0, 0.0, 9.81])
    
    # Get link6 body ID
    link6_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
    joint6_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "joint6")
    
    first_pass = True
    
    import time
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            if frame_idx >= num_frames:
                # Done - exit after first pass for system ID
                break
            
            # Step 1: Calculate feedforward torques (gravity compensation)
            data_guess.qpos[:7] = data.qpos[:7].copy()
            data_guess.qvel[:7] = 0
            data_guess.qacc[:7] = 0
            mujoco.mj_inverse(model_guess, data_guess)
            feedforward_torques = data_guess.qfrc_inverse[:7].copy()
            
            # Step 2: Set motor commands
            for i, motor in enumerate(motors):
                if i == 6:
                    # Joint 7 (index 6): LOCK at position 0
                    motor.set_mit_params(
                        kp=200.0,
                        kd=0.0,
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
            
            # Step 3: Update motor states
            for i, motor in enumerate(motors):
                motor.update(data.qpos[i], data.qvel[i])
            
            # Step 4: Compute and apply torques
            output_torques = np.zeros(7)
            for i, motor in enumerate(motors):
                output_torques[i] = motor.get_output_torque()
            data.ctrl[:7] = output_torques
            
            # ============================================================
            # LOG DATA FOR SYSTEM ID (link6 spatial velocity/acceleration)
            # ============================================================
            
            # Get spatial velocity of link6 at joint6 origin, in link6 frame
            # Using Jacobian method (same as get_cartesian_velocities)
            jacp = np.zeros((3, model.nv))
            jacr = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, jacp, jacr, joint6_site_id)
            
            # Velocity in world frame
            linear_vel_world = jacp @ data.qvel
            angular_vel_world = jacr @ data.qvel
            
            # Get link6 rotation matrix (world to link6: R.T transforms world to local)
            R_link6 = data.xmat[link6_body_id].reshape(3, 3)
            
            # Transform to link6 local frame
            angular_vel_local = R_link6.T @ angular_vel_world
            linear_vel_local = R_link6.T @ linear_vel_world
            
            # Current spatial velocity [w, v]
            current_spatial_vel = np.concatenate([angular_vel_local, linear_vel_local])
            
            # Log spatial velocity
            log_spatial_vel[frame_idx, :] = current_spatial_vel
            log_time[frame_idx] = frame_idx * dt
            log_torque[frame_idx] = data.ctrl[5]  # Joint6 commanded torque
            
            # Compute acceleration via finite difference (skip first frame)
            if prev_spatial_vel is not None:
                spatial_acc = (current_spatial_vel - prev_spatial_vel) / dt
                
                # Add gravity compensation to linear acceleration
                # Transform world gravity to link6 frame and add
                g_local = R_link6.T @ g_world
                spatial_acc[3:6] += g_local  # Add to linear acceleration part
                
                log_spatial_acc[frame_idx, :] = spatial_acc
            
            prev_spatial_vel = current_spatial_vel.copy()
            
            # Step simulation
            mujoco.mj_step(model, data)
            frame_idx += 1
            
            # Slow down playback
            time.sleep(dt * slowdown)
            
            # Print progress
            if frame_idx % 1000 == 0:
                print(f"Frame {frame_idx}/{num_frames} ({100*frame_idx/num_frames:.1f}%)")
            
            viewer.sync()
    
    # Skip first frame (no acceleration data)
    log_time = log_time[1:]
    log_spatial_vel = log_spatial_vel[1:]
    log_spatial_acc = log_spatial_acc[1:]
    log_torque = log_torque[1:]
    num_samples = len(log_time)
    
    # Low-pass filter acceleration to remove finite-diff noise
    # NOTE: When using real encoder data, this filtering will be essential
    from scipy.signal import butter, filtfilt
    def lowpass_filter(data, cutoff=50, fs=1000, order=2):
        b, a = butter(order, cutoff / (fs/2), btype='low')
        return filtfilt(b, a, data)
    
    for i in range(6):
        log_spatial_acc[:, i] = lowpass_filter(log_spatial_acc[:, i])
    
    # Save logged data to CSV
    log_file = os.path.join(os.path.dirname(__file__), "system_id_link6_data.csv")
    header = "time,wx,wy,wz,vx,vy,vz,dwx,dwy,dwz,dvx,dvy,dvz,tau6"
    log_data = np.zeros((num_samples, 14))
    log_data[:, 0] = log_time
    log_data[:, 1:7] = log_spatial_vel
    log_data[:, 7:13] = log_spatial_acc
    log_data[:, 13] = log_torque
    np.savetxt(log_file, log_data, delimiter=",", header=header, comments="")
    print(f"\nSaved system ID data to {log_file}")
    print(f"Collected {num_samples} samples over {log_time[-1]:.2f} seconds")

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
def generate_demo_trajectory(seconds: float, hz: int = 1000, test_joint: int = None, for_system_id: bool = False):
    """Generate oscillating motion on joints.
    
    Args:
        seconds: Duration of trajectory
        hz: Sample rate
        test_joint: If specified (0-6), only move this joint. If None, move all.
        for_system_id: If True, use multi-frequency excitation for better conditioning
    
    Returns:
        trajectory: (num_samples, 7) array of joint positions
        velocity: (num_samples, 7) array of joint velocities
    """
    num_samples = int(seconds * hz)
    t = np.linspace(0, seconds, num_samples)
    trajectory = np.zeros((num_samples, 7))
    velocity = np.zeros((num_samples, 7))
    
    if for_system_id:
        # Multi-frequency excitation for system identification
        # Sum of sinusoids for richer frequency content and better conditioning
        # Joint 6 (index 5) is active, joint 7 (index 6) is locked at 0
        
        # Base frequencies (Hz) - chosen to be incommensurate to avoid periodicity
        base_freqs = [0.31, 0.37, 0.43, 0.53, 0.59, 0.47, 0]
        amps = [0.4, 0.3, 0.25, 0.25, 0.15, 0.2, 0]  # Reduced amplitudes
        
        # Frequency multipliers for sum-of-sinusoids
        freq_multipliers = [1.0, 1.7, 2.3]
        amp_multipliers = [1.0, 0.4, 0.2]  # Reduced higher frequency contributions
        
        for j in range(6):  # Joints 0-5 (joint 6 stays at 0)
            if amps[j] == 0:
                continue
            for fm, am in zip(freq_multipliers, amp_multipliers):
                omega = 2 * np.pi * base_freqs[j] * fm
                # Use sin instead of cos so trajectory starts at 0
                trajectory[:, j] += amps[j] * am * np.sin(omega * t)
                velocity[:, j] += amps[j] * am * omega * np.cos(omega * t)
    else:
        # Original single-frequency trajectory
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
    elif len(sys.argv) > 1 and sys.argv[1] == "view_sysid":
        view_system_id_trajectory()
    else:
        system_id()
