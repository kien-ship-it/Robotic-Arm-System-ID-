"""
Debug: Check if regressor matches MuJoCo inverse dynamics.
"""
import mujoco
import numpy as np
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from generateRegressor import compute_joint_regressor_row


def main():
    model_path = os.path.join(os.path.dirname(__file__), "model", "kinova_fullinertia.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    link6_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
    joint6_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "joint6")
    joint6_axis = np.array([0, -1, 0])
    
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    g_world = np.array([0.0, 0.0, 9.81])
    
    # Hardcoded theta_true
    theta_true = np.array([
        0.789160, -0.000047, 0.009093, 0.120437,
        0.024382, 0.000001, 0.000009, 0.024312, -0.001638, 0.000551
    ])
    
    print("=" * 60)
    print("Test 1: Collect data and fit theta, then compare")
    print("=" * 60)
    
    # Collect data
    np.random.seed(42)
    n_samples = 500
    Y_list = []
    tau_mj_list = []
    tau_reg_list = []
    
    for i in range(n_samples):
        data.qpos[:7] = np.random.randn(7) * 0.5
        data.qvel[:7] = np.random.randn(7) * 2.0
        data.qacc[:7] = np.random.randn(7) * 5.0
        
        mujoco.mj_forward(model, data)
        
        # MuJoCo inverse
        mujoco.mj_inverse(model, data)
        tau_mj = data.qfrc_inverse[5]
        
        # Regressor
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
        
        tau_reg = Y_row @ theta_true
        
        Y_list.append(Y_row)
        tau_mj_list.append(tau_mj)
        tau_reg_list.append(tau_reg)
    
    Y = np.array(Y_list)
    tau_mj = np.array(tau_mj_list)
    tau_reg = np.array(tau_reg_list)
    
    # RMSE with hardcoded theta
    rmse_hardcoded = np.sqrt(np.mean((tau_mj - tau_reg)**2))
    print(f"RMSE with hardcoded theta_true: {rmse_hardcoded:.6f} Nm")
    
    # Fit theta to MuJoCo output
    theta_fit, _, rank, _ = np.linalg.lstsq(Y, tau_mj, rcond=1e-10)
    tau_fit = Y @ theta_fit
    rmse_fit = np.sqrt(np.mean((tau_mj - tau_fit)**2))
    print(f"RMSE with fitted theta: {rmse_fit:.6f} Nm")
    print(f"Matrix rank: {rank}")
    
    print("\n" + "=" * 60)
    print("Comparing theta_true vs theta_fit:")
    print("=" * 60)
    param_names = ['m', 'hx', 'hy', 'hz', 'Ixx', 'Ixy', 'Ixz', 'Iyy', 'Iyz', 'Izz']
    print(f"{'Param':<6} {'theta_true':>12} {'theta_fit':>12} {'diff':>12}")
    print("-" * 44)
    for name, t_true, t_fit in zip(param_names, theta_true, theta_fit):
        print(f"{name:<6} {t_true:>12.6f} {t_fit:>12.6f} {t_fit-t_true:>12.6f}")
    
    print("\n" + "=" * 60)
    print("Test 2: Simple static case (gravity only)")
    print("=" * 60)
    
    # Home position, no velocity, no acceleration
    data.qpos[:] = 0
    data.qvel[:] = 0
    data.qacc[:] = 0
    
    mujoco.mj_forward(model, data)
    mujoco.mj_inverse(model, data)
    tau_mj_static = data.qfrc_inverse[5]
    
    # Regressor for static case
    R = data.xmat[link6_body_id].reshape(3, 3)
    g_local = R.T @ g_world
    
    Y_row_static = compute_joint_regressor_row(
        0, 0, 0,  # w
        0, 0, 0,  # v
        0, 0, 0,  # dw
        g_local[0], g_local[1], g_local[2],  # dv = gravity in body frame
        joint6_axis
    )
    
    tau_reg_static = Y_row_static @ theta_true
    tau_reg_static_fit = Y_row_static @ theta_fit
    
    print(f"MuJoCo inverse (static): {tau_mj_static:.6f} Nm")
    print(f"Regressor (theta_true):  {tau_reg_static:.6f} Nm")
    print(f"Regressor (theta_fit):   {tau_reg_static_fit:.6f} Nm")
    print(f"Gravity in body frame: [{g_local[0]:.4f}, {g_local[1]:.4f}, {g_local[2]:.4f}]")
    
    print("\n" + "=" * 60)
    print("Test 3: Check what MuJoCo thinks the inertia is")
    print("=" * 60)
    
    # Print link6 inertia from model
    link6_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
    link7_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link7")
    
    print(f"Link6 mass: {model.body_mass[link6_id]:.6f}")
    print(f"Link6 ipos: {model.body_ipos[link6_id]}")
    print(f"Link6 inertia: {model.body_inertia[link6_id]}")
    
    print(f"\nLink7 mass: {model.body_mass[link7_id]:.6f}")
    print(f"Link7 ipos: {model.body_ipos[link7_id]}")
    print(f"Link7 inertia: {model.body_inertia[link7_id]}")


if __name__ == "__main__":
    main()
