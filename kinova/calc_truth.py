import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

m = mujoco.MjModel.from_xml_path('/Users/careycai/Desktop/7dof/7dof/kinova/model/kinova_fullinertia.xml')
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)

# Bodies to combine: link6 (6) and link7 (7)
# Note: we identify relative to joint6 site
site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'joint6')

# Need to compute everything in the site frame S.
m_total = 0
h_total = np.zeros(3)
I_total_S = np.zeros((3,3))

# site transform relative to world (at q=0)
p_S_W = d.site_xpos[site_id]
R_S_W = d.site_xmat[site_id].reshape(3,3)

for b_id in [6, 7]:
    mass = m.body_mass[b_id]
    # COM in world frame
    p_COM_W = d.xipos[b_id]
    # Inertia in world frame
    # (MuJoCo computes this in mj_forward: d.cinert contains everything in world frame?)
    # No, cinert is world-aligned but at COM.
    # Wait, d.cinert docs: "centered frame". 
    # Let's use the most reliable way: reconstruct from model params.
    
    # Orientation of body frame B in world frame
    R_B_W = d.xmat[b_id].reshape(3,3)
    # COM in body frame
    p_COM_B = m.body_ipos[b_id]
    # Orientation of inertial frame I relative to body frame B
    q_I_B = m.body_iquat[b_id]
    R_I_B = R.from_quat([q_I_B[1], q_I_B[2], q_I_B[3], q_I_B[0]]).as_matrix()
    # Principal moments of inertia
    I_principal = m.body_inertia[b_id]
    
    # Inertia tensor in body frame B
    I_B = R_I_B @ np.diag(I_principal) @ R_I_B.T
    
    # Now transform everything to the SITE frame S
    # p_site_B = site_pos in body frame? No, we have world positions.
    rel_pos_W = p_COM_W - p_S_W
    rel_pos_S = R_S_W.T @ rel_pos_W
    
    # First moment h = m*c in site frame
    h_total += mass * rel_pos_S
    m_total += mass
    
    # Rotate inertia to site frame S
    I_S = R_S_W.T @ (R_B_W @ I_B @ R_B_W.T) @ R_S_W
    
    # Parallel Axis theorem: shift I from COM to site origin
    d_vec = rel_pos_S
    d_sq = np.dot(d_vec, d_vec)
    I_S_shifted = I_S + mass * (d_sq * np.eye(3) - np.outer(d_vec, d_vec))
    
    I_total_S += I_S_shifted

print(f"Total Mass: {m_total:.6f}")
print(f"Total h: {h_total}")
print(f"Total I (at site):\n{I_total_S}")

# [m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
theta = [
    m_total, h_total[0], h_total[1], h_total[2],
    I_total_S[0,0], I_total_S[0,1], I_total_S[0,2],
    I_total_S[1,1], I_total_S[1,2], I_total_S[2,2]
]
print(f"\nTheta for system_id.py:")
print("[" + ", ".join([f"{v:.6f}" for v in theta]) + "]")
