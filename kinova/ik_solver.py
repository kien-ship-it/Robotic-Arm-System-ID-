"""
Pinocchio-based Inverse Kinematics Solver for Kinova 7-DOF Arm

Uses damped least-squares (Levenberg-Marquardt) method for numerical IK.
"""

import numpy as np
import pinocchio as pin
from pathlib import Path


class KinovaIKSolver:
    """Inverse kinematics solver for Kinova arm using Pinocchio."""
    
    def __init__(self, urdf_path: str = None):
        """
        Initialize the IK solver with the robot model.
        
        Args:
            urdf_path: Path to URDF file. Defaults to kinova.urdf in urdf/ folder.
        """
        if urdf_path is None:
            urdf_path = Path(__file__).parent / "urdf" / "kinova.urdf"
        
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        
        # End-effector frame (link_tool)
        self.ee_frame_id = self.model.getFrameId("link_tool")
        
        # Default solver parameters
        self.max_iter = 200
        self.eps = 1e-4          # Convergence threshold
        self.dt = 0.1            # Integration step
        self.damp = 1e-6         # Damping factor for pseudo-inverse
        
    def forward_kinematics(self, q: np.ndarray) -> pin.SE3:
        """
        Compute forward kinematics for given joint configuration.
        
        Args:
            q: Joint angles (7,)
            
        Returns:
            SE3 pose of end-effector
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.ee_frame_id].copy()
    
    def solve(self, target_pose: pin.SE3, q_init: np.ndarray = None,
              position_only: bool = False) -> tuple[np.ndarray, bool, dict]:
        """
        Solve inverse kinematics for target pose.
        
        Args:
            target_pose: Desired SE3 pose of end-effector
            q_init: Initial joint configuration. Defaults to neutral pose.
            position_only: If True, only match position (ignore orientation)
            
        Returns:
            Tuple of (joint_angles, success, info_dict)
        """
        if q_init is None:
            q_init = pin.neutral(self.model)
        
        q = q_init.copy()
        
        for i in range(self.max_iter):
            # Compute current pose
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            current_pose = self.data.oMf[self.ee_frame_id]
            
            # Compute error in SE3
            error_se3 = pin.log6(current_pose.actInv(target_pose))
            error = error_se3.vector
            
            if position_only:
                error = error[:3]  # Only position error
            
            # Check convergence
            if np.linalg.norm(error) < self.eps:
                return q, True, {"iterations": i + 1, "error": np.linalg.norm(error)}
            
            # Compute Jacobian
            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )
            
            if position_only:
                J = J[:3, :]  # Only position rows
            
            # Damped least-squares (Levenberg-Marquardt)
            JJt = J @ J.T
            JJt_damped = JJt + self.damp * np.eye(JJt.shape[0])
            dq = J.T @ np.linalg.solve(JJt_damped, error)
            
            # Update joint angles
            q = pin.integrate(self.model, q, self.dt * dq)
            
            # Clamp to joint limits
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
        
        # Failed to converge
        final_error = np.linalg.norm(error)
        return q, False, {"iterations": self.max_iter, "error": final_error}
    
    def solve_position(self, target_position: np.ndarray, 
                       q_init: np.ndarray = None) -> tuple[np.ndarray, bool, dict]:
        """
        Solve IK for position only (orientation free).
        
        Args:
            target_position: Desired [x, y, z] position
            q_init: Initial joint configuration
            
        Returns:
            Tuple of (joint_angles, success, info_dict)
        """
        # Create SE3 with target position and identity rotation
        target_pose = pin.SE3(np.eye(3), np.array(target_position))
        return self.solve(target_pose, q_init, position_only=True)
    
    def solve_with_orientation(self, position: np.ndarray, 
                                rotation: np.ndarray,
                                q_init: np.ndarray = None) -> tuple[np.ndarray, bool, dict]:
        """
        Solve IK for full 6-DOF pose.
        
        Args:
            position: Desired [x, y, z] position
            rotation: 3x3 rotation matrix
            q_init: Initial joint configuration
            
        Returns:
            Tuple of (joint_angles, success, info_dict)
        """
        target_pose = pin.SE3(rotation, np.array(position))
        return self.solve(target_pose, q_init, position_only=False)


def create_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Create rotation matrix from RPY angles (radians)."""
    return pin.rpy.rpyToMatrix(roll, pitch, yaw)


if __name__ == "__main__":
    # Demo usage
    solver = KinovaIKSolver()
    
    print(f"Robot: {solver.model.name}")
    print(f"DOF: {solver.model.nq}")
    print(f"End-effector frame: link_tool (id={solver.ee_frame_id})")
    print()
    
    # Get home position FK
    q_home = pin.neutral(solver.model)
    home_pose = solver.forward_kinematics(q_home)
    print(f"Home position FK:")
    print(f"  Position: {home_pose.translation}")
    print(f"  Rotation:\n{home_pose.rotation}")
    print()
    
    # Test IK: move to a target position
    target_pos = np.array([0.3, 0.1, 0.5])
    print(f"Solving IK for position: {target_pos}")
    
    q_sol, success, info = solver.solve_position(target_pos)
    print(f"  Success: {success}")
    print(f"  Iterations: {info['iterations']}")
    print(f"  Final error: {info['error']:.6f}")
    
    if success:
        result_pose = solver.forward_kinematics(q_sol)
        print(f"  Achieved position: {result_pose.translation}")
        print(f"  Joint angles (deg): {np.degrees(q_sol)}")
    print()
    
    # Test full pose IK
    target_rot = create_rotation_matrix(0, np.pi/4, 0)  # 45° pitch
    print(f"Solving IK for full pose (position + 45° pitch)")
    
    q_sol2, success2, info2 = solver.solve_with_orientation(target_pos, target_rot)
    print(f"  Success: {success2}")
    print(f"  Iterations: {info2['iterations']}")
    print(f"  Final error: {info2['error']:.6f}")
