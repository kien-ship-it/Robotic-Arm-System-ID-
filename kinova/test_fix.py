"""
Quick test to verify the dv correction fixes the double-counting issue.
"""
import mujoco
import numpy as np
import os
from generateRegressor import compute_joint_regressor_row

model_path = os.path.join(os.path.dirname(__file__), "model", "kinova_fullinertia.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

theta_true = np.array([
    0.789160, -0.000047, 0.009093, 0.120437,
    0.024382, 0.000001, 0.000009, 0.024312, -0.001638, 0.000551
])

joint6_axis = np.array([0, -1, 0])
link6_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'link6')
joint6_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'joint6')
g_world = np.array([0, 0, 9.81])

jacp = np.zeros((3, model.nv))
jacr = np.zeros((3, model.nv))
dt = model.opt.timestep

print("Testing with dv correction (dv_local = R.T @ dv_world - w x v)")
print("=" * 60)

errors_corrected = []
errors_uncorrected = []

np.random.seed(42)
n_tests = 100

for i in range(n_tests):
    # Random configuration with velocity ONLY on joint 6
    # Other joints stationary to isolate link6+7 dynamics
    data.qpos[:7] = np.random.uniform(-0.5, 0.5, 7)
    data.qvel[:7] = 0
    data.qvel[5] = np.random.uniform(-2, 2)  # Only joint 6 moving
    mujoco.mj_forward(model, data)
    
    # Apply random torque to joint 6
    data.ctrl[:7] = 0
    data.ctrl[5] = np.random.uniform(-2, 2)
    tau_applied = data.ctrl[5]
    
    # Get velocity before
    mujoco.mj_jacSite(model, data, jacp, jacr, joint6_site_id)
    v_before = jacp @ data.qvel
    w_before = jacr @ data.qvel
    R_before = data.xmat[link6_body_id].reshape(3, 3).copy()
    
    # Step
    mujoco.mj_step(model, data)
    
    # Get velocity after
    mujoco.mj_jacSite(model, data, jacp, jacr, joint6_site_id)
    v_after = jacp @ data.qvel
    w_after = jacr @ data.qvel
    
    # Acceleration in world frame
    dv_world = (v_after - v_before) / dt + g_world
    dw_world = (w_after - w_before) / dt
    
    # Project to body frame
    w_local = R_before.T @ w_before
    v_local = R_before.T @ v_before
    dw_local = R_before.T @ dw_world
    
    # UNCORRECTED (old way)
    dv_local_uncorrected = R_before.T @ dv_world
    
    # CORRECTED (new way)
    dv_local_corrected = R_before.T @ dv_world - np.cross(w_local, v_local)
    
    # Regressor predictions
    Y_row_uncorrected = compute_joint_regressor_row(
        w_local[0], w_local[1], w_local[2],
        v_local[0], v_local[1], v_local[2],
        dw_local[0], dw_local[1], dw_local[2],
        dv_local_uncorrected[0], dv_local_uncorrected[1], dv_local_uncorrected[2],
        joint6_axis
    )
    
    Y_row_corrected = compute_joint_regressor_row(
        w_local[0], w_local[1], w_local[2],
        v_local[0], v_local[1], v_local[2],
        dw_local[0], dw_local[1], dw_local[2],
        dv_local_corrected[0], dv_local_corrected[1], dv_local_corrected[2],
        joint6_axis
    )
    
    tau_uncorrected = Y_row_uncorrected @ theta_true
    tau_corrected = Y_row_corrected @ theta_true
    
    errors_uncorrected.append(tau_applied - tau_uncorrected)
    errors_corrected.append(tau_applied - tau_corrected)

errors_uncorrected = np.array(errors_uncorrected)
errors_corrected = np.array(errors_corrected)

print(f"\nUNCORRECTED (old way):")
print(f"  RMSE: {np.sqrt(np.mean(errors_uncorrected**2)):.6f} Nm")
print(f"  Mean: {errors_uncorrected.mean():.6f} Nm")

print(f"\nCORRECTED (new way):")
print(f"  RMSE: {np.sqrt(np.mean(errors_corrected**2)):.6f} Nm")
print(f"  Mean: {errors_corrected.mean():.6f} Nm")
