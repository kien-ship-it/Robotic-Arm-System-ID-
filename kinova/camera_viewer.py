"""
Kinova arm simulation with live wrist camera viewer.
Uses OpenCV for real-time camera feed display.

Note: On macOS, OpenCV GUI must run on main thread.
"""
import mujoco
import mujoco.viewer
import numpy as np
import cv2
import os
import time
import threading

# Load model
model_path = os.path.join(os.path.dirname(__file__), "model", "kinova.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Create renderer for wrist camera
cam_width, cam_height = 640, 480
renderer = mujoco.Renderer(model, height=cam_height, width=cam_width)

# Set initial joint positions
data.qpos[:] = [0, 0, 0, 0, 0, 0, 0]
mujoco.mj_forward(model, data)

# Shared state
frame_lock = threading.Lock()
current_frame = None
running = True
viewer_handle = None


def simulation_thread():
    """Thread to run MuJoCo simulation and viewer."""
    global current_frame, running, viewer_handle
    
    frame_count = 0
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer_handle = viewer
        while viewer.is_running() and running:
            step_start = time.time()
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Update camera feed every 10 steps (~100Hz at 1kHz sim)
            frame_count += 1
            if frame_count % 10 == 0:
                renderer.update_scene(data, camera="wrist_cam")
                frame = renderer.render()
                
                with frame_lock:
                    current_frame = frame.copy()
            
            viewer.sync()
            
            # Realtime sync
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)
    
    running = False


print("Simulation running with live wrist camera feed.")
print("Press 'Q' in camera window to quit.")
print("Close MuJoCo viewer to quit.")

# Start simulation in background thread
sim_thread = threading.Thread(target=simulation_thread, daemon=True)
sim_thread.start()

# Run OpenCV display on main thread (required for macOS)
cv2.namedWindow("Wrist Camera", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Wrist Camera", cam_width, cam_height)

while running:
    with frame_lock:
        frame = current_frame.copy() if current_frame is not None else None
    
    if frame is not None:
        # Convert RGB to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("Wrist Camera", frame_bgr)
    
    # Check for 'q' key to quit
    key = cv2.waitKey(30) & 0xFF
    if key == ord('q'):
        running = False
        break

cv2.destroyAllWindows()
sim_thread.join(timeout=1.0)
renderer.close()
print("Done.")
