import sys
import os
import pytest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from forward_kinematics_2d import fk_2link_planar
from inverse_kinematics_2d import ik_2link_planar, ik_3link_planar
from forward_kinematics_6dof import RobotArm6DOF
from inverse_kinematics_6dof import NumericalIKSolver6DOF, analytical_ik_spherical_wrist
from dh_transform import extract_pose


def test_2link_ik_consistency():
    l1, l2 = 1.0, 0.7
    targets = [
        (1.2, 0.4),
        (-0.8, 0.9),
        (0.5, -0.6)
    ]
    for xt, yt in targets:
        sols = ik_2link_planar(l1, l2, xt, yt)
        for config_name in ["elbow_down", "elbow_up"]:
            sol = sols[config_name]
            assert sol is not None, f"Expected reachable solution for {xt}, {yt}"
            th1, th2 = sol
            ee_pose, _ = fk_2link_planar(l1, l2, th1, th2)
            assert np.allclose(ee_pose[:2], [xt, yt], atol=1e-5)


def test_2link_ik_unreachable():
    l1, l2 = 1.0, 0.5
    # Beyond outer radius l1 + l2 = 1.5
    far_res = ik_2link_planar(l1, l2, 1.8, 0.0)
    assert far_res["elbow_down"] is None
    assert far_res["elbow_up"] is None
    
    # Inside inner radius |l1 - l2| = 0.5
    inner_res = ik_2link_planar(l1, l2, 0.2, 0.0)
    assert inner_res["elbow_down"] is None
    assert inner_res["elbow_up"] is None


def test_6dof_numerical_ik_convergence():
    arm = RobotArm6DOF()
    solver = NumericalIKSolver6DOF(arm, pos_tolerance=1e-3, rot_tolerance=1e-2, max_iterations=100)
    
    # Generate test poses using forward kinematics
    test_configs = [
        np.radians([15.0, 30.0, -20.0, 10.0, -35.0, 25.0]),
        np.radians([-20.0, 45.0, -15.0, -10.0, 20.0, -15.0]),
        np.radians([35.0, 20.0, -40.0, 25.0, -20.0, 10.0])
    ]
    
    for q_true in test_configs:
        T_target, _ = arm.forward_kinematics(q_true)
        target_pos, target_rot = extract_pose(T_target)
        
        # Seed slightly perturbed
        q_seed = q_true + np.random.uniform(-0.15, 0.15, 6)
        
        success, q_sol, iters, pos_err, rot_err = solver.solve(
            target_pos, target_rot, q_initial=q_seed
        )
        
        assert success, f"IK failed to converge for target {target_pos} (err={pos_err:.4f})"
        assert pos_err < 1e-3
        assert rot_err < 1e-2
        
        # Verify FK of solution matches target
        T_sol, _ = arm.forward_kinematics(q_sol)
        assert np.allclose(T_sol[:3, 3], target_pos, atol=2e-3)


def test_6dof_numerical_ik_unreachable_target():
    arm = RobotArm6DOF()
    solver = NumericalIKSolver6DOF(arm, max_iterations=50)
    
    # 10 metres away is physically outside the 1.05m arm reach
    target_pos = np.array([5.0, 5.0, 5.0])
    target_rot = np.eye(3)
    
    success, _, iters, pos_err, _ = solver.solve(target_pos, target_rot)
    assert not success, "Solver should report failure on out-of-reach target"
    assert pos_err > 1.0


def test_6dof_joint_limits_respected():
    arm = RobotArm6DOF()
    solver = NumericalIKSolver6DOF(arm)
    
    target_pos = np.array([0.4, 0.3, 0.6])
    target_rot = np.eye(3)
    
    success, q_sol, iters, pos_err, _ = solver.solve(target_pos, target_rot)
    for j in range(6):
        assert arm.joint_limits[j, 0] <= q_sol[j] <= arm.joint_limits[j, 1]
