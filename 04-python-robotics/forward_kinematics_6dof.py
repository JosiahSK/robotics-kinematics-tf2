"""
Forward Kinematics for a 6-DOF Serial Robotic Manipulator
Author: Josiah & Educational Robotics Contributors
License: MIT

This module implements the standard Denavit-Hartenberg (DH) forward kinematics
for a representative 6-DOF industrial/educational robotic manipulator (EduArm-6DOF).
It computes all intermediate link frames, validates orthogonality and determinant
of the rotation matrices, and provides coordinate frames for visualization.
"""

from typing import List, Tuple, Union, Optional
import numpy as np
from dh_transform import dh_matrix, extract_pose


class RobotArm6DOF:
    """
    Kinematic model of an illustrative 6-DOF articulated serial robot arm.
    
    Architecture:
        - 6 revolute joints (RRRRRR)
        - Anthropomorphic arm with spherical wrist (axes 4, 5, 6 intersect at wrist center)
        
    DH Parameters (Standard DH Convention):
        Link 1: theta1 (var), d1 = 0.400 m, a1 = 0.000 m, alpha1 = +pi/2 (+90 deg)
        Link 2: theta2 (var), d2 = 0.000 m, a2 = 0.500 m, alpha2 =  0.000   (0 deg)
        Link 3: theta3 (var), d3 = 0.000 m, a3 = 0.000 m, alpha3 = +pi/2 (+90 deg)
        Link 4: theta4 (var), d4 = 0.450 m, a4 = 0.000 m, alpha4 = -pi/2 (-90 deg)
        Link 5: theta5 (var), d5 = 0.000 m, a5 = 0.000 m, alpha5 = +pi/2 (+90 deg)
        Link 6: theta6 (var), d6 = 0.100 m, a6 = 0.000 m, alpha6 =  0.000   (0 deg)
        
    Note:
        These parameters represent a standard illustrative academic model
        designed for rigorous study and are not claimed to represent any
        specific proprietary commercial manipulator.
    """
    def __init__(
        self,
        d1: float = 0.400,
        a2: float = 0.500,
        d4: float = 0.450,
        d6: float = 0.100
    ):
        self.d1 = float(d1)
        self.a2 = float(a2)
        self.d4 = float(d4)
        self.d6 = float(d6)
        
        # Joint limits in radians [min, max]
        self.joint_limits = np.array([
            [-np.pi,        np.pi],        # Joint 1: Base waist
            [-np.pi / 2,    np.pi / 2],    # Joint 2: Shoulder
            [-np.pi * 0.8,  np.pi * 0.8],  # Joint 3: Elbow
            [-np.pi,        np.pi],        # Joint 4: Wrist roll
            [-np.pi / 2,    np.pi / 2],    # Joint 5: Wrist pitch
            [-np.pi,        np.pi]         # Joint 6: Wrist roll / tool
        ], dtype=float)

    def get_dh_table(self, joint_angles: np.ndarray) -> List[Tuple[float, float, float, float]]:
        """
        Return list of standard DH tuples (theta_i, d_i, a_i, alpha_i).
        """
        q = np.asarray(joint_angles, dtype=float)
        if q.shape != (6,):
            raise ValueError(f"Expected 6 joint angles, got shape {q.shape}.")
            
        return [
            (q[0], self.d1, 0.0,      np.pi / 2),
            (q[1], 0.0,     self.a2,  0.0),
            (q[2], 0.0,     0.0,      np.pi / 2),
            (q[3], self.d4, 0.0,     -np.pi / 2),
            (q[4], 0.0,     0.0,      np.pi / 2),
            (q[5], self.d6, 0.0,      0.0)
        ]

    def forward_kinematics(
        self,
        joint_angles: np.ndarray,
        in_degrees: bool = False
    ) -> Tuple[np.ndarray, List[np.ndarray]]:
        """
        Compute forward kinematics T_0^6 = A_1 * A_2 * A_3 * A_4 * A_5 * A_6.
        
        Parameters:
            joint_angles: 6 joint values (radians by default, degrees if in_degrees=True).
            in_degrees: If True, input angles are in degrees.
            
        Returns:
            Tuple[T_0_6, intermediate_transforms]:
                - T_0_6: 4x4 homogeneous transformation matrix of end-effector relative to base.
                - intermediate_transforms: List of 7 transformation matrices [T_0^0, T_0^1, ..., T_0^6].
        """
        q = np.asarray(joint_angles, dtype=float)
        if in_degrees:
            q = np.deg2rad(q)
            
        dh_params = self.get_dh_table(q)
        
        T_curr = np.eye(4, dtype=float)
        transforms = [T_curr.copy()]
        
        for theta, d, a, alpha in dh_params:
            A_i = dh_matrix(theta=theta, d=d, a=a, alpha=alpha, in_degrees=False)
            T_curr = T_curr @ A_i
            transforms.append(T_curr.copy())
            
        T_0_6 = T_curr
        
        # Verify rotation matrix properties
        R = T_0_6[:3, :3]
        det_R = np.linalg.det(R)
        ortho_check = np.allclose(R.T @ R, np.eye(3), atol=1e-6)
        
        if not (ortho_check and np.isclose(det_R, 1.0, atol=1e-5)):
            raise ValueError(
                f"Numerical degradation in FK rotation matrix: det={det_R:.6f}, orthogonal={ortho_check}"
            )
            
        return T_0_6, transforms

    def get_joint_positions(self, joint_angles: np.ndarray, in_degrees: bool = False) -> np.ndarray:
        """
        Get the 3D origin coordinates of all frames (0 to 6) in the base frame.
        Useful for 3D plotting and link wireframes.
        
        Returns:
            np.ndarray: shape (7, 3) containing [p0, p1, p2, p3, p4, p5, p6].
        """
        _, transforms = self.forward_kinematics(joint_angles, in_degrees=in_degrees)
        positions = np.array([T[:3, 3] for T in transforms], dtype=float)
        return positions

    def get_wrist_center(self, joint_angles: np.ndarray, in_degrees: bool = False) -> np.ndarray:
        """
        Extract the wrist center position (frame 4/5 origin, where axes 4, 5, 6 intersect).
        """
        _, transforms = self.forward_kinematics(joint_angles, in_degrees=in_degrees)
        return transforms[4][:3, 3].copy()


def urdf_to_dh_joints(q_urdf: np.ndarray) -> np.ndarray:
    """
    Convert joint angles from URDF coordinate frame conventions to standard DH model conventions.
    
    The URDF model uses standard robotics visualization conventions:
    - Zero pose: arm is outstretched horizontally along the +X axis (TCP at [1.05, 0, 0.4]).
    - Joint 2 (shoulder pitch) and Joint 3 (elbow pitch) rotate about +Y.
    - Joint 5 (wrist pitch) rotates about +Y.
    
    The canonical DH model uses standard Denavit-Hartenberg parameterization:
    - Shoulder and elbow DH axes z1, z2 point along -Y_base.
    - Elbow DH zero angle has link 4 pointing along -Z_base (arm bent at 90 deg).
    - Straight horizontal configuration corresponds to q_dh[2] = +pi/2.
    
    Mapping:
        q_dh[0] =  q_urdf[0]
        q_dh[1] = -q_urdf[1]
        q_dh[2] =  pi/2 - q_urdf[2]
        q_dh[3] =  q_urdf[3]
        q_dh[4] = -q_urdf[4]
        q_dh[5] =  q_urdf[5]
    """
    q = np.asarray(q_urdf, dtype=float).copy()
    if q.shape != (6,):
        raise ValueError(f"Expected 6 joint angles, got shape {q.shape}")
    return np.array([
        q[0],
        -q[1],
        np.pi / 2.0 - q[2],
        q[3],
        -q[4],
        q[5]
    ], dtype=float)


def dh_to_urdf_joints(q_dh: np.ndarray) -> np.ndarray:
    """
    Convert joint angles from standard DH model conventions to URDF coordinate frame conventions.
    
    Inverse mapping of urdf_to_dh_joints:
        q_urdf[0] =  q_dh[0]
        q_urdf[1] = -q_dh[1]
        q_urdf[2] =  pi/2 - q_dh[2]
        q_urdf[3] =  q_dh[3]
        q_urdf[4] = -q_dh[4]
        q_urdf[5] =  q_dh[5]
    """
    q = np.asarray(q_dh, dtype=float).copy()
    if q.shape != (6,):
        raise ValueError(f"Expected 6 joint angles, got shape {q.shape}")
    return np.array([
        q[0],
        -q[1],
        np.pi / 2.0 - q[2],
        q[3],
        -q[4],
        q[5]
    ], dtype=float)


def dh_to_urdf_flange_rotation() -> np.ndarray:
    """
    Return the 3x3 rotation matrix mapping DH flange orientation to URDF TCP orientation:
        R_urdf = R_dh @ R_flange_offset
    where R_flange_offset maps the DH end-effector frame (z pointing forward along tool,
    x pointing up) to URDF TCP frame (x pointing forward along tool, z pointing up).
    """
    return np.array([
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0]
    ], dtype=float)


def urdf_forward_kinematics(
    q_urdf: np.ndarray,
    arm: Optional[RobotArm6DOF] = None
) -> Tuple[np.ndarray, List[np.ndarray]]:
    """
    Compute forward kinematics using the URDF joint frame origins and rotation axes.
    
    Parameters:
        q_urdf: 6 joint angles in URDF convention (radians).
        arm: Optional RobotArm6DOF instance providing dimension parameters (d1, a2, d4, d6).
        
    Returns:
        Tuple[T_tcp, link_transforms]:
            - T_tcp: 4x4 transform of tool_center_point relative to base_link.
            - link_transforms: List of 4x4 transforms for [base_link, link_1, link_2, link_3, link_4, link_5, tool_flange, tool_center_point].
    """
    if arm is None:
        arm = RobotArm6DOF()
        
    q = np.asarray(q_urdf, dtype=float)
    if q.shape != (6,):
        raise ValueError(f"Expected 6 joint angles, got shape {q.shape}")
        
    from rotation_matrices import rot_x, rot_y, rot_z

    # Base to Link 1 (Joint 1: origin [0, 0, d1], axis [0, 0, 1])
    T_0_1 = np.eye(4, dtype=float)
    T_0_1[:3, 3] = [0.0, 0.0, arm.d1]
    T_0_1[:3, :3] = rot_z(q[0])

    # Link 1 to Link 2 (Joint 2: origin [0, 0, 0], axis [0, 1, 0])
    T_1_2 = np.eye(4, dtype=float)
    T_1_2[:3, :3] = rot_y(q[1])

    # Link 2 to Link 3 (Joint 3: origin [a2, 0, 0], axis [0, 1, 0])
    T_2_3 = np.eye(4, dtype=float)
    T_2_3[:3, 3] = [arm.a2, 0.0, 0.0]
    T_2_3[:3, :3] = rot_y(q[2])

    # Link 3 to Link 4 (Joint 4: origin [0, 0, 0], axis [1, 0, 0])
    T_3_4 = np.eye(4, dtype=float)
    T_3_4[:3, :3] = rot_x(q[3])

    # Link 4 to Link 5 (Joint 5: origin [d4, 0, 0], axis [0, 1, 0])
    T_4_5 = np.eye(4, dtype=float)
    T_4_5[:3, 3] = [arm.d4, 0.0, 0.0]
    T_4_5[:3, :3] = rot_y(q[4])

    # Link 5 to Tool Flange (Joint 6: origin [0, 0, 0], axis [1, 0, 0])
    T_5_6 = np.eye(4, dtype=float)
    T_5_6[:3, :3] = rot_x(q[5])

    # Tool Flange to Tool Center Point (Fixed joint: origin [d6, 0, 0])
    T_6_tcp = np.eye(4, dtype=float)
    T_6_tcp[:3, 3] = [arm.d6, 0.0, 0.0]

    # Accumulate chain
    T_0 = np.eye(4, dtype=float)
    T_1 = T_0 @ T_0_1
    T_2 = T_1 @ T_1_2
    T_3 = T_2 @ T_2_3
    T_4 = T_3 @ T_3_4
    T_5 = T_4 @ T_4_5
    T_6 = T_5 @ T_5_6
    T_tcp = T_6 @ T_6_tcp

    return T_tcp, [T_0, T_1, T_2, T_3, T_4, T_5, T_6, T_tcp]


if __name__ == "__main__":
    print("=" * 60)
    print("6-DOF Arm Forward Kinematics Verification")
    print("=" * 60)
    
    arm = RobotArm6DOF()
    
    # Test 1: Zero configuration (Home Pose)
    q_zero = np.zeros(6)
    T_zero, all_T = arm.forward_kinematics(q_zero)
    pos_zero, rot_zero = extract_pose(T_zero)
    
    print("1. Zero configuration (q = [0, 0, 0, 0, 0, 0]):")
    print(f"   End-Effector Position (m): {np.round(pos_zero, 4)}")
    print(f"   End-Effector Orientation R:\n{np.round(rot_zero, 4)}")
    print(f"   det(R) = {np.linalg.det(rot_zero):.6f}")
    
    # Test 2: Articulated configuration
    q_test = np.array([30.0, 45.0, -30.0, 20.0, -45.0, 15.0])
    T_test, _ = arm.forward_kinematics(q_test, in_degrees=True)
    pos_test, rot_test = extract_pose(T_test)
    
    print("\n2. Articulated configuration (degrees: [30, 45, -30, 20, -45, 15]):")
    print(f"   End-Effector Position (m): {np.round(pos_test, 4)}")
    print(f"   det(R) = {np.linalg.det(rot_test):.6f}")
    print(f"   Wrist Center: {np.round(arm.get_wrist_center(q_test, in_degrees=True), 4)}")
    print("=" * 60)
