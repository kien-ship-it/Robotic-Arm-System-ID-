"""
Kinova arm simulation with live wrist camera viewer.
Uses DearPyGui for real-time camera feed display.

Run with regular python (not mjpython):
    python camera_viewer.py
"""
import mujoco
import numpy as np
import os
import time
import dearpygui.dearpygui as dpg
from mujoco_renderer import MujocoRenderer

# Load model
model_path = os.path.join(os.path.dirname(__file__), "model", "kinova.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Create renderer
renderer = MujocoRenderer(model)

# Set initial joint positions
data.qpos[:] = [0.03, 0.824, -0.487, -0.928, 0.673, -2.116, 0.534]
mujoco.mj_forward(model, data)

# Setup DearPyGui
dpg.create_context()

# Create textures for both views
with dpg.texture_registry():
    blank_main = np.zeros((renderer.main_height, renderer.main_width, 4), dtype=np.float32)
    blank_cam = np.zeros((renderer.cam_height, renderer.cam_width, 4), dtype=np.float32)
    dpg.add_raw_texture(renderer.main_width, renderer.main_height, blank_main, format=dpg.mvFormat_Float_rgba, tag="main_texture")
    dpg.add_raw_texture(renderer.cam_width, renderer.cam_height, blank_cam, format=dpg.mvFormat_Float_rgba, tag="cam_texture")

# Main window with both views
with dpg.window(label="Kinova Simulation", tag="main_window"):
    with dpg.group(horizontal=True):
        with dpg.group():
            dpg.add_text("Main View")
            dpg.add_image("main_texture")
        with dpg.group():
            dpg.add_text("Wrist Camera")
            dpg.add_image("cam_texture")
    
    dpg.add_separator()
    dpg.add_text("Camera Info")
    dpg.add_text("Position: ", tag="cam_pos_text")
    dpg.add_text("View Direction: ", tag="cam_dir_text")
    
    dpg.add_separator()
    dpg.add_text("Joint Controls")
    
    joint_names = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]
    for i, name in enumerate(joint_names):
        dpg.add_slider_float(
            label=name, 
            default_value=float(data.qpos[i]),
            min_value=-3.14, 
            max_value=3.14,
            tag=f"joint_{i}",
            width=300
        )

dpg.create_viewport(title="Kinova Camera Viewer", width=1500, height=750)
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.set_primary_window("main_window", True)

print("Simulation running with live camera feed.")
print("Use sliders to control joints. Close window to quit.")

frame_count = 0
last_time = time.time()

while dpg.is_dearpygui_running():
    for i in range(7):
        data.qpos[i] = dpg.get_value(f"joint_{i}")
    
    mujoco.mj_step(model, data)
    
    frame_count += 1
    # Render every 80 frames to maintain ~12.5Hz renderer frequency (1000Hz / 80 = 12.5Hz)
    if frame_count % 80 == 0:
        # Render views
        main_rgba = renderer.render_main_view(data)
        dpg.set_value("main_texture", main_rgba.flatten())
        
        cam_rgba = renderer.render_wrist_camera(data)
        dpg.set_value("cam_texture", cam_rgba.flatten())
        
        # Update camera info
        cam_pos, cam_forward = renderer.get_camera_pose(data)
        dpg.set_value("cam_pos_text", f"Position: x={cam_pos[0]:.3f}, y={cam_pos[1]:.3f}, z={cam_pos[2]:.3f}")
        dpg.set_value("cam_dir_text", f"View Direction: x={cam_forward[0]:.3f}, y={cam_forward[1]:.3f}, z={cam_forward[2]:.3f}")
    
    dpg.render_dearpygui_frame()
    
    # Target 1000Hz (0.001 second per frame)
    elapsed = time.time() - last_time
    if elapsed < 0.001:
        time.sleep(0.001 - elapsed)
    last_time = time.time()

dpg.destroy_context()
renderer.close()
print("Done.")
