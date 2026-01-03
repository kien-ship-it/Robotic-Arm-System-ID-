"""Test MuJoCo with Kinova XML"""
import mujoco
from mujoco import viewer
from pathlib import Path

# Load model
xml_path = Path(__file__).parent / "model" / "kinova.xml"
model = mujoco.MjModel.from_xml_path(str(xml_path))
data = mujoco.MjData(model)

print(f"Model timestep: {model.opt.timestep}")
print(f"Joints: {model.nq}")
print(f"Actuators: {model.nu}")

# Launch viewer
viewer.launch(model, data)
