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

# Load model
model_path = os.path.join(os.path.dirname(__file__), "model", "kinova.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Find joint body IDs (Kinova model uses link1-link7)
joint_body_names = [f"link{i}" for i in range(1, 8)]
body_ids = []
for name in joint_body_names:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id >= 0:
        body_ids.append((name, body_id))

print(f"Tracking {len(body_ids)} bodies: {[name for name, _ in body_ids]}")

def get_cartesian_velocities(model, data):
    """Compute Cartesian velocity at each joint body using Jacobians."""
    velocities = []
    
    for name, body_id in body_ids:
        # Allocate Jacobians
        jacp = np.zeros((3, model.nv))  # translational
        jacr = np.zeros((3, model.nv))  # rotational
        
        # Compute Jacobian for this body
        mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
        
        # Cartesian velocity = J @ qvel
        linear_vel = jacp @ data.qvel
        angular_vel = jacr @ data.qvel
        
        velocities.append({
            'name': name,
            'body_id': body_id,
            'linear': linear_vel.copy(),
            'angular': angular_vel.copy(),
            'speed': np.linalg.norm(linear_vel)
        })
    
    return velocities

def format_velocity_display(velocities):
    """Format velocities for display."""
    lines = ["Cartesian Velocities (m/s):", ""]
    for v in velocities:
        lv = v['linear']
        lines.append(f"{v['name']:>8}: [{lv[0]:+6.3f}, {lv[1]:+6.3f}, {lv[2]:+6.3f}]  |v|={v['speed']:.3f}")
    return "\n".join(lines)

# Generate a simple oscillating trajectory for demo
def generate_demo_trajectory(seconds: float, hz: int = 1000):
    """Generate oscillating motion on first 4 joints."""
    num_samples = int(seconds * hz)
    t = np.linspace(0, seconds, num_samples)
    trajectory = np.zeros((num_samples, 7))
    
    # Different frequencies for each joint
    freqs = [0.3, 0.4, 0.5, 0.6]
    amps = [0.5, 0.4, 0.3, 0.3]
    
    for j in range(4):
        trajectory[:, j] = amps[j] * np.sin(2 * np.pi * freqs[j] * t)
    
    return trajectory

if __name__ == "__main__":
    print("=" * 60)
    print("Kinova Cartesian Velocity Viewer")
    print("=" * 60)
    print("Shows real-time Cartesian velocity at each joint body.")
    print("Drag the arm or watch the trajectory playback.")
    print("=" * 60)
    
    # Generate trajectory
    trajectory = generate_demo_trajectory(seconds=10.0)
    frame_idx = 0
    num_frames = trajectory.shape[0]
    prev_qpos = trajectory[0, :].copy()
    
    # For numerical velocity estimation
    dt = 0.001  # 1ms timestep
    
    import time
    last_print = 0
    print_interval = 0.05  # Print every 50ms
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # Update position from trajectory
            if frame_idx < num_frames:
                data.qpos[:7] = trajectory[frame_idx, :]
                # Estimate velocity numerically
                data.qvel[:7] = (trajectory[frame_idx, :] - prev_qpos) / dt
                prev_qpos = trajectory[frame_idx, :].copy()
                frame_idx += 1
            else:
                # Loop trajectory
                frame_idx = 0
                prev_qpos = trajectory[0, :].copy()
            
            mujoco.mj_forward(model, data)
            
            # Compute Cartesian velocities
            velocities = get_cartesian_velocities(model, data)
            
            # Print formatted table at intervals
            now = time.time()
            if now - last_print > print_interval:
                last_print = now
                # Clear screen and print table
                print("\033[2J\033[H")  # Clear screen, cursor to top
                print("=" * 70)
                print("  CARTESIAN VELOCITIES AT EACH JOINT (m/s)")
                print("=" * 70)
                print(f"{'Body':<10} {'vx':>10} {'vy':>10} {'vz':>10} {'|v|':>10}")
                print("-" * 70)
                for v in velocities:
                    lv = v['linear']
                    print(f"{v['name']:<10} {lv[0]:>+10.4f} {lv[1]:>+10.4f} {lv[2]:>+10.4f} {v['speed']:>10.4f}")
                print("-" * 70)
                print(f"Frame: {frame_idx}/{num_frames}")
            
            viewer.sync()
    
    print("\n\nDone!")
