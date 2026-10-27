import sys
import os
import pytest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from forward_kinematics_2d import fk_2link_planar, fk_2link_dh, fk_3link_planar
from forward_kinematics_6dof import RobotArm6DOF
from dh_transform import extract_pose


def test_2link_planar_fk():
    l1, l2 = 1.0, 0.5
    th1, th2 = np.pi / 4, np.pi / 4
    
    ee_pose, joints = fk_2link_planar(l1, l2, th1, th2)
    
    # Expected: x = l1*cos(45) + l2*cos(90) = 1*sqrt(2)/2 + 0 = 0.7071
    # y = l1*sin(45) + l2*sin(90) = 0.7071 + 0.5 = 1.2071
    expected_x = l1 * np.cos(np.pi / 4) + l2 * np.cos(np.pi / 2)
    expected_y = l1 * np.sin(np.pi / 4) + l2 * np.sin(np.pi / 2)
    
    assert np.isclose(ee_pose[0], expected_x, atol=1e-6)
    assert np.isclose(ee_pose[1], expected_y, atol=1e-6)
    assert np.isclose(ee_pose[2], np.pi / 2, atol=1e-6)
    
    # Cross verify with DH formulation
    T_dh = fk_2link_dh(l1, l2, th1, th2)
    assert np.allclose(ee_pose[:2], T_dh[:2, 3], atol=1e-6)


def test_6dof_arm_home_position():
    arm = RobotArm6DOF(d1=0.400, a2=0.500, d4=0.450, d6=0.100)
    q_zero = np.zeros(6)
    
    T_0_6, transforms = arm.forward_kinematics(q_zero)
    pos, rot = extract_pose(T_0_6)
    
    # At q=0:
    # Joint 1: z0 up, x1 forward
    # Joint 2: rotated 0, link length 0.5 along x2
    # Joint 3: alpha=+pi/2, link 4 has d4=0.45
    # Let's check wrist center at q=0:
    p_wc = arm.get_wrist_center(q_zero)
    assert np.isclose(p_wc[0], 0.5, atol=1e-4)
    # In our DH convention, z_3 points in -Z base direction, so z = d1 - d4 = 0.4 - 0.45 = -0.05
    assert np.isclose(p_wc[2], arm.d1 - arm.d4, atol=1e-4)
    
    # Check rotation matrix validity
    assert np.isclose(np.linalg.det(rot), 1.0, atol=1e-6)
    assert np.allclose(rot.T @ rot, np.eye(3), atol=1e-6)


def test_6dof_arm_random_configurations_orthogonality():
    arm = RobotArm6DOF()
    np.random.seed(123)
    for _ in range(10):
        q_rand = np.random.uniform(-np.pi / 2, np.pi / 2, 6)
        T, _ = arm.forward_kinematics(q_rand)
        R = T[:3, :3]
        assert np.isclose(np.linalg.det(R), 1.0, atol=1e-5)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-5)
