import mujoco
try:
    model = mujoco.MjModel.from_xml_path("./7DOF_Arm.xml")
    print(f"Successfully loaded model.")
    print(f"Number of joints (nq): {model.nq}")
    print(f"Number of DOFs (nv): {model.nv}")
    print(f"Joint names: {[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]}")
except Exception as e:
    print(f"Failed to load model: {e}")
