"""
Inverse Kinematics for a 6-DOF Serial Manipulator
Author: Josiah & Educational Robotics Contributors
License: MIT

This module provides two distinct inverse kinematics implementations:
1. Analytical IK: Decoupled wrist-centre method for anthropomorphic 6-DOF arms
   with spherical wrists (axes 4, 5, 6 intersecting at a point).
2. Numerical IK: Damped Least Squares (Levenberg-Marquardt) Jacobian solver
   with exact Lie-algebra SO(3) rotational error, joint limit handling,
   and convergence diagnostics.
"""

from typing import Dict, List, Optional, Tuple, Union
import numpy as np
from forward_kinematics_6dof import RobotArm6DOF
from jacobian import geometric_jacobian_6dof, damped_least_squares_inverse
from dh_transform import dh_matrix, extract_pose


def rotation_error_vector(R_current: np.ndarray, R_target: np.ndarray) -> np.ndarray:
    """
    Compute mathematically consistent 3D orientation error vector e_o in SO(3).
    
    Uses the skew-symmetric cross-product formulation:
        e_o = 0.5 * (n x n_d + s x s_d + a x a_d)
    where R_current = [n, s, a] and R_target = [n_d, s_d, a_d].
    
    This avoids Euler angle discontinuities, gimbal lock, and arbitrary angle wrapping.
    """
    n, s, a = R_current[:, 0], R_current[:, 1], R_current[:, 2]
    nd, sd, ad = R_target[:, 0], R_target[:, 1], R_target[:, 2]
    
    e_o = 0.5 * (np.cross(n, nd) + np.cross(s, sd) + np.cross(a, ad))
    return e_o


class NumericalIKSolver6DOF:
    """
    Numerical Inverse Kinematics solver using Damped Least Squares (DLS).
    """
    def __init__(
        self,
        arm: RobotArm6DOF,
        pos_tolerance: float = 1e-4,     # 0.1 mm
        rot_tolerance: float = 1e-3,     # ~0.05 deg
        max_iterations: int = 150,
        damping: float = 0.05,
        step_limit: float = 0.25          # Maximum joint step per iteration (rad)
    ):
        self.arm = arm
        self.pos_tol = float(pos_tolerance)
        self.rot_tol = float(rot_tolerance)
        self.max_iter = int(max_iterations)
        self.damping = float(damping)
        self.step_limit = float(step_limit)

    def solve(
        self,
        target_pos: np.ndarray,
        target_rot: np.ndarray,
        q_initial: Optional[np.ndarray] = None
    ) -> Tuple[bool, np.ndarray, int, float, float]:
        """
        Iteratively solve for joint angles q to achieve target position and orientation.
        
        Parameters:
            target_pos: Target 3D position [x, y, z] in metres.
            target_rot: Target 3x3 rotation matrix in SO(3).
            q_initial: Optional initial joint seed (defaults to zero pose).
            
        Returns:
            Tuple[success, q_solution, iterations, pos_error, rot_error]:
                - success: True if converged within tolerances.
                - q_solution: Computed 6 joint angles in radians.
                - iterations: Number of iterations executed.
                - pos_error: Final Euclidean position error (m).
                - rot_error: Final angular orientation error norm.
        """
        target_pos = np.asarray(target_pos, dtype=float).flatten()
        target_rot = np.asarray(target_rot, dtype=float)
        
        if q_initial is None:
            q = np.array([0.1, 0.2, -0.2, 0.1, 0.1, 0.1], dtype=float)
        else:
            q = np.asarray(q_initial, dtype=float).copy()
            
        pos_err_norm = float("inf")
        rot_err_norm = float("inf")
        
        for iteration in range(self.max_iter):
            # 1. Forward Kinematics at current configuration
            T_curr, _ = self.arm.forward_kinematics(q)
            curr_pos = T_curr[:3, 3]
            curr_rot = T_curr[:3, :3]
            
            # 2. Compute error vectors
            e_p = target_pos - curr_pos
            e_o = rotation_error_vector(curr_rot, target_rot)
            
            pos_err_norm = float(np.linalg.norm(e_p))
            rot_err_norm = float(np.linalg.norm(e_o))
            
            # Check convergence
            if pos_err_norm < self.pos_tol and rot_err_norm < self.rot_tol:
                return True, q, iteration, pos_err_norm, rot_err_norm
                
            error_vector = np.concatenate([e_p, e_o])
            
            # Check for non-finite values (NaN / Inf protection)
            if not np.all(np.isfinite(error_vector)):
                return False, q, iteration, pos_err_norm, rot_err_norm
                
            # 3. Geometric Jacobian
            J = geometric_jacobian_6dof(self.arm, q)
            
            # 4. Damped Least Squares update
            J_dls = damped_least_squares_inverse(J, damping=self.damping)
            delta_q = J_dls @ error_vector
            
            # Clamp maximum joint step for numerical stability
            step_norm = np.linalg.norm(delta_q)
            if step_norm > self.step_limit:
                delta_q = delta_q * (self.step_limit / step_norm)
                
            # 5. Update joint angles
            q += delta_q
            
            # 6. Respect joint limits
            for j in range(6):
                q[j] = np.clip(q[j], self.arm.joint_limits[j, 0], self.arm.joint_limits[j, 1])
                
        # If loop finishes without meeting tolerance:
        return False, q, self.max_iter, pos_err_norm, rot_err_norm


def analytical_ik_spherical_wrist(
    arm: RobotArm6DOF,
    target_pos: np.ndarray,
    target_rot: np.ndarray
) -> List[np.ndarray]:
    """
    Closed-form analytical Inverse Kinematics via kinematic decoupling
    (Wrist-Centre Method) for the illustrative EduArm-6DOF.
    
    Geometry Assumptions:
        1. Robot has 6 revolute joints.
        2. Three consecutive joint axes (joints 4, 5, 6) intersect at a single point (wrist center).
        3. Joint 1 axis is vertical along base Z.
        4. Joints 2 and 3 axes are parallel to each other and perpendicular to joint 1 axis.
        
    Steps:
        1. Decouple position: Find wrist center p_wc = p_ee - d6 * R_target * [0, 0, 1]^T.
        2. Solve inverse position kinematics (theta1, theta2, theta3) for p_wc.
        3. Decouple orientation: Compute R_0^3(theta1, theta2, theta3).
        4. Compute wrist rotation R_3^6 = (R_0^3)^T * R_target.
        5. Solve inverse orientation kinematics (theta4, theta5, theta6) from R_3^6.
        
    Returns:
        List of valid joint angle solutions np.ndarray of shape (6,).
    """
    target_pos = np.asarray(target_pos, dtype=float).flatten()
    target_rot = np.asarray(target_rot, dtype=float)
    
    # Tool axis z_tool in base frame is column 2 of target_rot
    z_tool = target_rot[:, 2]
    
    # 1. Wrist center position
    p_wc = target_pos - arm.d6 * z_tool
    x_w, y_w, z_w = p_wc
    
    solutions = []
    
    # 2. Joint 1: Base rotation
    # Two possibilities for base: forward and backward (+pi)
    for theta1 in [np.arctan2(y_w, x_w), np.arctan2(-y_w, -x_w)]:
        # Horizontal and vertical coordinates in arm plane
        r = np.sqrt(x_w**2 + y_w**2)
        if np.isclose(theta1, np.arctan2(-y_w, -x_w)):
            r = -r
            
        s = z_w - arm.d1
        D_sq = r**2 + s**2
        
        # Law of cosines for link 2 (a2) and forearm (d4)
        cos_theta3 = (D_sq - arm.a2**2 - arm.d4**2) / (2.0 * arm.a2 * arm.d4)
        if abs(cos_theta3) > 1.0:
            continue  # Target wrist center position is unreachable
            
        # Two elbow configurations: Elbow-up and Elbow-down
        sin_theta3_pos = np.sqrt(max(0.0, 1.0 - cos_theta3**2))
        for sin_theta3 in [sin_theta3_pos, -sin_theta3_pos]:
            theta3 = np.arctan2(sin_theta3, cos_theta3)
            
            # Shoulder angle theta2
            theta2 = np.arctan2(s, r) - np.arctan2(arm.d4 * sin_theta3, arm.a2 + arm.d4 * cos_theta3)
            
            # 3. Compute R_0^3 from (theta1, theta2, theta3)
            A1 = dh_matrix(theta=theta1, d=arm.d1, a=0.0, alpha=np.pi/2)
            A2 = dh_matrix(theta=theta2, d=0.0, a=arm.a2, alpha=0.0)
            A3 = dh_matrix(theta=theta3, d=0.0, a=0.0, alpha=np.pi/2)
            T_0_3 = A1 @ A2 @ A3
            R_0_3 = T_0_3[:3, :3]
            
            # 4. Wrist orientation R_3^6 = (R_0^3)^T @ R_target
            R_3_6 = R_0_3.T @ target_rot
            
            # 5. Extract Euler angles (theta4, theta5, theta6) from R_3_6
            # Standard spherical wrist parameterization:
            # R_3_6 = R_z(theta4) * R_x(-pi/2) * R_z(theta5) * R_x(pi/2) * R_z(theta6)
            # From DH: A4(theta4, d4, 0, -pi/2), A5(theta5, 0, 0, pi/2), A6(theta6, d6, 0, 0)
            # The rotation part of A4*A5*A6 gives:
            # R_3_6[2, 2] = cos(theta5)
            cos_th5 = np.clip(R_3_6[2, 2], -1.0, 1.0)
            
            sin_th5_pos = np.sqrt(max(0.0, 1.0 - cos_th5**2))
            for sin_th5 in [sin_th5_pos, -sin_th5_pos]:
                theta5 = np.arctan2(sin_th5, cos_th5)
                
                if np.isclose(sin_th5, 0.0, atol=1e-5):
                    # Wrist singularity (theta5 = 0): theta4 and theta6 are collinear
                    theta4 = 0.0
                    theta6 = np.arctan2(R_3_6[1, 0], R_3_6[0, 0])
                else:
                    theta4 = np.arctan2(R_3_6[1, 2], R_3_6[0, 2])
                    theta6 = np.arctan2(R_3_6[2, 1], -R_3_6[2, 0])
                    
                candidate = np.array([theta1, theta2, theta3, theta4, theta5, theta6], dtype=float)
                
                # Check that candidate satisfies forward kinematics
                T_check, _ = arm.forward_kinematics(candidate)
                pos_err = np.linalg.norm(T_check[:3, 3] - target_pos)
                rot_err = np.linalg.norm(rotation_error_vector(T_check[:3, :3], target_rot))
                
                if pos_err < 1e-3 and rot_err < 1e-3:
                    # Check joint limits
                    in_limits = True
                    for j in range(6):
                        if not (arm.joint_limits[j, 0] <= candidate[j] <= arm.joint_limits[j, 1]):
                            in_limits = False
                            break
                    if in_limits:
                        solutions.append(candidate)
                        
    return solutions


if __name__ == "__main__":
    print("=" * 60)
    print("6-DOF Arm Inverse Kinematics Verification")
    print("=" * 60)
    
    arm = RobotArm6DOF()
    solver = NumericalIKSolver6DOF(arm)
    
    # 1. Forward-Inverse Consistency Test
    q_known = np.radians([25.0, 35.0, -30.0, 15.0, -40.0, 20.0])
    T_target, _ = arm.forward_kinematics(q_known)
    target_pos, target_rot = extract_pose(T_target)
    
    print("Known Joint Angles (deg): [25, 35, -30, 15, -40, 20]")
    print(f"Target Position: {np.round(target_pos, 4)}")
    
    # Test Numerical IK
    success, q_num, iters, pos_err, rot_err = solver.solve(target_pos, target_rot)
    print(f"\nNumerical IK Result:")
    print(f"  Success: {success} (in {iters} iterations)")
    print(f"  Position Error: {pos_err:.2e} m")
    print(f"  Orientation Error: {rot_err:.2e}")
    print(f"  Computed Angles (deg): {np.round(np.degrees(q_num), 2)}")
    
    # Verify FK matches
    T_result, _ = arm.forward_kinematics(q_num)
    reproduced_pos = T_result[:3, 3]
    print(f"  FK Position from Solution: {np.round(reproduced_pos, 4)}")
    print(f"  Verified exact match? {np.allclose(reproduced_pos, target_pos, atol=1e-3)}")
    
    # Test Unreachable Target Handling
    unreachable_pos = np.array([5.0, 5.0, 5.0])  # Far outside reach
    success_unreach, _, iters_u, pos_err_u, _ = solver.solve(unreachable_pos, target_rot)
    print(f"\nUnreachable Target Test (5, 5, 5):")
    print(f"  Reported Failure correctly? {not success_unreach}")
    print(f"  Remaining error: {pos_err_u:.2f} m")
    print("=" * 60)
