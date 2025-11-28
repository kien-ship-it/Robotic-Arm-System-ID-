import mujoco
import numpy as np
from mujoco import viewer
import time

model = mujoco.MjModel.from_xml_path("./5DOF_Arm.xml")
data = mujoco.MjData(model)

def compute_static_torques(model, data, qpos):
    # Set state
    data.qpos[:] = qpos
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0

    # Update model
    mujoco.mj_forward(model, data)
    
    # Reset qacc to 0 to compute holding torque (otherwise mj_inverse uses the falling acceleration from mj_forward)
    data.qacc[:] = 0.0
    data.qvel[:] = 0.0

    # Compute inverse dynamics for this static pose (gravity + bias)
    mujoco.mj_inverse(model, data)
    return np.copy(data.qfrc_inverse)

def ik(qpos):
    # qpos = np.deg2rad(qpos_deg)[:model.nq]
    # Compute once
    torques = compute_static_torques(model, data, qpos)
    # print("Required torques:", torques)
    # Set the visual pose so you can inspect it
    return torques
    # Just show it, don't step physics (so it won't 'fall')
    # with viewer.launch_passive(model, data) as v:
    #     while v.is_running():
    #         time.sleep(0.05)
