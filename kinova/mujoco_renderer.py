"""
MuJoCo rendering utilities for the Kinova arm simulation.
"""
import mujoco
import numpy as np


class MujocoRenderer:
    """Handles MuJoCo rendering for main view and wrist camera."""
    
    def __init__(self, model, main_size=(800, 600), cam_size=(640, 480)):
        self.model = model
        self.main_width, self.main_height = main_size
        self.cam_width, self.cam_height = cam_size
        
        # Create renderers
        self.main_renderer = mujoco.Renderer(model, height=self.main_height, width=self.main_width)
        self.cam_renderer = mujoco.Renderer(model, height=self.cam_height, width=self.cam_width)
        
        # Main view camera setup
        self.main_cam = mujoco.MjvCamera()
        self.main_cam.azimuth = 135
        self.main_cam.elevation = -20
        self.main_cam.distance = 2.5
        self.main_cam.lookat[:] = [0, 0, 0.5]
        
        # Get wrist camera ID
        self.wrist_cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_cam")
    
    def render_main_view(self, data):
        """Render the main view and return RGBA float array."""
        self.main_renderer.update_scene(data, self.main_cam)
        frame = self.main_renderer.render()
        rgba = np.dstack([frame, np.full((self.main_height, self.main_width), 255, dtype=np.uint8)])
        return rgba.astype(np.float32) / 255.0
    
    def render_wrist_camera(self, data):
        """Render the wrist camera view and return RGBA float array."""
        self.cam_renderer.update_scene(data, camera="wrist_cam")
        frame = self.cam_renderer.render()
        rgba = np.dstack([frame, np.full((self.cam_height, self.cam_width), 255, dtype=np.uint8)])
        return rgba.astype(np.float32) / 255.0
    
    def get_camera_pose(self, data):
        """Get wrist camera position and viewing direction."""
        cam_pos = data.cam_xpos[self.wrist_cam_id].copy()
        cam_mat = data.cam_xmat[self.wrist_cam_id].reshape(3, 3)
        cam_forward = -cam_mat[:, 2]  # viewing direction (into scene)
        return cam_pos, cam_forward
    
    def close(self):
        """Clean up renderers."""
        self.main_renderer.close()
        self.cam_renderer.close()
