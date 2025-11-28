
import numpy as np
import mujoco
from ik import ik

if __name__ == "__main__":
    # Test pose from user logs
    test_qpos = np.array([1.178, -0.771, 0.466, 0.0])
    print(f"Testing IK with qpos: {test_qpos}")
    
    torques = ik(test_qpos)
    print(f"Raw Torques: {torques}")
