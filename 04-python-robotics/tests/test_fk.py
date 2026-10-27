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
    rng = np.random.default_rng(123)
    for _ in range(10):
        q_rand = rng.uniform(-np.pi / 2, np.pi / 2, 6)
        T, _ = arm.forward_kinematics(q_rand)
        R = T[:3, :3]
        assert np.isclose(np.linalg.det(R), 1.0, atol=1e-5)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-5)


def test_6dof_known_configurations():
    """Verify forward kinematics against hand-derived geometric poses."""
    arm = RobotArm6DOF(d1=0.400, a2=0.500, d4=0.450, d6=0.100)

    # 1. Arm outstretched horizontally along +X: q = [0, 0, 90, 0, 0, 0] deg
    T_horiz, _ = arm.forward_kinematics(np.radians([0, 0, 90, 0, 0, 0]))
    assert np.allclose(T_horiz[:3, 3], [1.050, 0.000, 0.400], atol=1e-4)

    # 2. Arm outstretched horizontally along +Y: q = [90, 0, 90, 0, 0, 0] deg
    T_y, _ = arm.forward_kinematics(np.radians([90, 0, 90, 0, 0, 0]))
    assert np.allclose(T_y[:3, 3], [0.000, 1.050, 0.400], atol=1e-4)

    # 3. Arm pointing straight up along +Z: q = [0, 90, 90, 0, 0, 0] deg
    T_up, _ = arm.forward_kinematics(np.radians([0, 90, 90, 0, 0, 0]))
    assert np.allclose(T_up[:3, 3], [0.000, 0.000, 1.450], atol=1e-4)


def test_urdf_dh_parameter_consistency():
    """Verify that the URDF XML dimensions match the canonical DH parameters."""
    import xml.etree.ElementTree as ET
    urdf_path = os.path.join(os.path.dirname(__file__), "..", "..", "05-ros2-practical", "urdf", "eduarm_6dof.urdf")
    assert os.path.exists(urdf_path), f"URDF file not found at {urdf_path}"

    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joints_by_name = {j.attrib["name"]: j for j in root.findall("joint")}

    # Joint 1: height d1 = 0.400m
    j1_xyz = [float(x) for x in joints_by_name["joint_1"].find("origin").attrib["xyz"].split()]
    assert np.isclose(j1_xyz[2], 0.400)

    # Joint 3: distance a2 = 0.500m
    j3_xyz = [float(x) for x in joints_by_name["joint_3"].find("origin").attrib["xyz"].split()]
    assert np.isclose(j3_xyz[0], 0.500)

    # Joint 5: distance d4 = 0.450m
    j5_xyz = [float(x) for x in joints_by_name["joint_5"].find("origin").attrib["xyz"].split()]
    assert np.isclose(j5_xyz[0], 0.450)

    # TCP: distance d6 = 0.100m
    tcp_xyz = [float(x) for x in joints_by_name["tcp_joint"].find("origin").attrib["xyz"].split()]
    assert np.isclose(tcp_xyz[0], 0.100)

