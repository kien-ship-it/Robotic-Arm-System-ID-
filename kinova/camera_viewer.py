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

# Load model
model_path = os.path.join(os.path.dirname(__file__), "model", "kinova.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Create renderer for wrist camera
cam_width, cam_height = 640, 480
renderer = mujoco.Renderer(model, height=cam_height, width=cam_width)

# Create renderer for main view
main_width, main_height = 800, 600
main_renderer = mujoco.Renderer(model, height=main_height, width=main_width)

# Set initial joint positions
data.qpos[:] = [0.03, 0.824, -0.487, -0.928, 0.673, -2.116, 0.534]
mujoco.mj_forward(model, data)

# Camera for main view
main_cam = mujoco.MjvCamera()
main_cam.azimuth = 135
main_cam.elevation = -20
main_cam.distance = 2.5
main_cam.lookat[:] = [0, 0, 0.5]

# Setup DearPyGui
dpg.create_context()

# Create textures for both views
with dpg.texture_registry():
    # Initialize with blank frames (RGBA format, values 0-1)
    blank_main = np.zeros((main_height, main_width, 4), dtype=np.float32)
    blank_cam = np.zeros((cam_height, cam_width, 4), dtype=np.float32)
    dpg.add_raw_texture(main_width, main_height, blank_main, format=dpg.mvFormat_Float_rgba, tag="main_texture")
    dpg.add_raw_texture(cam_width, cam_height, blank_cam, format=dpg.mvFormat_Float_rgba, tag="cam_texture")

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
    dpg.add_text("Joint Controls")
    
    # Joint sliders
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
    # Read joint values from sliders
    for i in range(7):
        data.qpos[i] = dpg.get_value(f"joint_{i}")
    
    # Step simulation
    mujoco.mj_step(model, data)
    
    # Update displays every few frames
    frame_count += 1
    if frame_count % 5 == 0:
        # Render main view
        main_renderer.update_scene(data, main_cam)
        main_frame = main_renderer.render()
        # Convert to RGBA float [0,1]
        main_rgba = np.dstack([main_frame, np.full((main_height, main_width), 255, dtype=np.uint8)])
        main_rgba = main_rgba.astype(np.float32) / 255.0
        dpg.set_value("main_texture", main_rgba.flatten())
        
        # Render wrist camera
        renderer.update_scene(data, camera="wrist_cam")
        cam_frame = renderer.render()
        # Convert to RGBA float [0,1]
        cam_rgba = np.dstack([cam_frame, np.full((cam_height, cam_width), 255, dtype=np.uint8)])
        cam_rgba = cam_rgba.astype(np.float32) / 255.0
        dpg.set_value("cam_texture", cam_rgba.flatten())
    
    dpg.render_dearpygui_frame()
    
    # Throttle to ~60fps
    elapsed = time.time() - last_time
    if elapsed < 0.016:
        time.sleep(0.016 - elapsed)
    last_time = time.time()

dpg.destroy_context()
renderer.close()
main_renderer.close()
print("Done.")
