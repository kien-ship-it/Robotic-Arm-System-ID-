"""
Full validation of the system ID pipeline:
1. Run a realistic excitation trajectory in simulation
2. Collect (Y, tau) data using MuJoCo inverse dynamics as ground truth
3. Fit theta from the data
4. Test on a NEW trajectory and compare regressor vs MuJoCo

Run with: mjpython validate_sysid_pipeline.py
"""
import mujoco
import mujoco.viewer
import numpy as np
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from generateRegressor import compute_joint_regressor_row


def generate_smooth_trajectory(duration=10.0, hz=1000, seed=42):
    """Generate smooth multi-frequency trajectory for all joints."""
    np.random.seed(seed)
    n_samples = int(duration * hz)
    t = np.linspace(0, duration, n_samples)
    
    trajectory = np.zeros((n_samples, 7))
    velocity = np.zeros((n_samples, 7))
    acceleration = np.zeros((n_samples, 7))
    
    # Each joint: sum of 3-4 sinusoids with random freq/amp/phase
    for j in range(7):
        n_sines = np.random.randint(3, 5)
        for _ in range(n_sines):
            freq = np.random.uniform(0.1, 0.6)  # Low frequencies for smooth motion
            amp = np.random.uniform(0.15, 0.35)
            phase = np.random.uniform(0, 2*np.pi)
            omega = 2 * np.pi * freq
            
            trajectory[:, j] += amp * np.sin(omega * t + phase)
            velocity[:, j] += amp * omega * np.cos(omega * t + phase)
            acceleration[:, j] -= amp * omega**2 * np.sin(omega * t + phase)
    
    return trajectory, velocity, acceleration, t


def collect_sysid_data(model, data, trajectory, velocity, acceleration, t_array):
    """
    Collect system ID data along a trajectory.
    Returns regressor matrix Y and MuJoCo inverse dynamics torques.
    """
    link6_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
    joint6_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "joint6")
    joint6_axis = np.array([0, -1, 0])
    
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    g_world = np.array([0.0, 0.0, 9.81])
    
    n_samples = len(t_array)
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
        
        # Kinematics
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_crb(model, data)
        
        # Get Jacobian and velocity
        mujoco.mj_jacSite(model, data, jacp, jacr, joint6_site_id)
        v_world = jacp @ qvel
        w_world = jacr @ qvel
        
        R = data.xmat[link6_body_id].reshape(3, 3)
        w_local = R.T @ w_world
        v_local = R.T @ v_world
        
        # Compute Jdot @ qvel via finite difference
        data.qpos[:7] = qpos + eps * qvel
        mujoco.mj_kinematics(model, data)
        
        jacp_plus = np.zeros((3, model.nv))
        jacr_plus = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp_plus, jacr_plus, joint6_site_id)
        
        Jdot_v_linear = (jacp_plus - jacp) @ qvel / eps
        Jdot_v_angular = (jacr_plus - jacr) @ qvel / eps
        
        # Full acceleration
        dv_world = jacp @ qacc + Jdot_v_linear
        dw_world = jacr @ qacc + Jdot_v_angular
        
        # Transform to body frame
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
        
        # Restore and get MuJoCo inverse dynamics
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
        
        if i % 2000 == 0:
            print(f"  Collecting: {i}/{n_samples} ({100*i/n_samples:.0f}%)")
    
    return np.array(Y_list), np.array(tau_mj_list)


def main():
    print("=" * 60)
    print("System ID Pipeline Validation")
    print("=" * 60)
    
    # Load model
    model_path = os.path.join(os.path.dirname(__file__), "model", "kinova_fullinertia.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # ========================================
    # STEP 1: Generate training trajectory
    # ========================================
    print("\n[Step 1] Generating training trajectory...")
    train_traj, train_vel, train_acc, train_t = generate_smooth_trajectory(
        duration=15.0, hz=1000, seed=42
    )
    print(f"  Duration: {train_t[-1]:.1f}s, {len(train_t)} samples")
    
    # ========================================
    # STEP 2: Collect training data
    # ========================================
    print("\n[Step 2] Collecting training data (regressor + MuJoCo inverse)...")
    Y_train, tau_train = collect_sysid_data(
        model, data, train_traj, train_vel, train_acc, train_t
    )
    print(f"  Y shape: {Y_train.shape}")
    print(f"  tau shape: {tau_train.shape}")
    print(f"  tau range: [{tau_train.min():.3f}, {tau_train.max():.3f}] Nm")
    
    # ========================================
    # STEP 3: Fit parameters
    # ========================================
    print("\n[Step 3] Fitting inertial parameters...")
    theta_fit, _, rank, singular_values = np.linalg.lstsq(Y_train, tau_train, rcond=1e-10)
    
    print(f"  Matrix rank: {rank}")
    print(f"  Condition number: {np.linalg.cond(Y_train):.2f}")
    
    # Training RMSE
    tau_train_pred = Y_train @ theta_fit
    train_rmse = np.sqrt(np.mean((tau_train - tau_train_pred)**2))
    print(f"  Training RMSE: {train_rmse:.6f} Nm")
    
    # Compare with ground truth theta
    theta_true = np.array([
        0.789160, -0.000047, 0.009093, 0.120437,
        0.024382, 0.000001, 0.000009, 0.024312, -0.001638, 0.000551
    ])
    
    print("\n  Fitted vs Ground Truth parameters:")
    param_names = ['m', 'hx', 'hy', 'hz', 'Ixx', 'Ixy', 'Ixz', 'Iyy', 'Iyz', 'Izz']
    print(f"  {'Param':<6} {'Fitted':>12} {'True':>12} {'Diff':>12}")
    print("  " + "-" * 44)
    for name, fit, true in zip(param_names, theta_fit, theta_true):
        print(f"  {name:<6} {fit:>12.6f} {true:>12.6f} {fit-true:>12.6f}")
    
    # ========================================
    # STEP 4: Test on NEW trajectory
    # ========================================
    print("\n[Step 4] Testing on NEW trajectory...")
    test_traj, test_vel, test_acc, test_t = generate_smooth_trajectory(
        duration=10.0, hz=1000, seed=123  # Different seed!
    )
    print(f"  Duration: {test_t[-1]:.1f}s, {len(test_t)} samples")
    
    print("  Collecting test data...")
    Y_test, tau_test = collect_sysid_data(
        model, data, test_traj, test_vel, test_acc, test_t
    )
    
    # Predict with fitted theta
    tau_test_pred = Y_test @ theta_fit
    
    # Metrics
    test_rmse = np.sqrt(np.mean((tau_test - tau_test_pred)**2))
    test_max_err = np.max(np.abs(tau_test - tau_test_pred))
    test_mean_err = np.mean(np.abs(tau_test - tau_test_pred))
    
    # Relative error
    tau_range = tau_test.max() - tau_test.min()
    rel_rmse = test_rmse / tau_range * 100
    
    print(f"\n  Test Results:")
    print(f"    RMSE: {test_rmse:.6f} Nm")
    print(f"    Max error: {test_max_err:.6f} Nm")
    print(f"    Mean error: {test_mean_err:.6f} Nm")
    print(f"    Torque range: [{tau_test.min():.3f}, {tau_test.max():.3f}] Nm")
    print(f"    Relative RMSE: {rel_rmse:.2f}%")
    
    # ========================================
    # STEP 5: Save results for plotting
    # ========================================
    print("\n[Step 5] Saving results...")
    results = np.column_stack([test_t, tau_test, tau_test_pred])
    np.savetxt(
        os.path.join(os.path.dirname(__file__), "sysid_validation_results.csv"),
        results,
        delimiter=",",
        header="time,tau_mujoco,tau_regressor",
        comments=""
    )
    print("  Saved to sysid_validation_results.csv")
    
    # ========================================
    # Summary
    # ========================================
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"Training RMSE: {train_rmse:.6f} Nm")
    print(f"Test RMSE:     {test_rmse:.6f} Nm")
    print(f"Relative RMSE: {rel_rmse:.2f}%")
    
    if test_rmse < 0.1:
        print("\n✓ Pipeline VALIDATED - regressor matches MuJoCo inverse dynamics")
    else:
        print("\n✗ Pipeline needs investigation - RMSE too high")
    
    return theta_fit, test_rmse


if __name__ == "__main__":
    theta_fit, rmse = main()
