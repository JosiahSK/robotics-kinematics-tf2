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


# ─────────────────────── 2D PLANAR IK TESTS ────────────────────────

def test_2link_ik_consistency():
    l1, l2 = 1.0, 0.7
    targets = [(1.2, 0.4), (-0.8, 0.9), (0.5, -0.6)]
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
    far_res = ik_2link_planar(l1, l2, 1.8, 0.0)  # Beyond outer radius 1.5
    assert far_res["elbow_down"] is None
    assert far_res["elbow_up"] is None
    inner_res = ik_2link_planar(l1, l2, 0.2, 0.0)  # Inside inner radius 0.5
    assert inner_res["elbow_down"] is None
    assert inner_res["elbow_up"] is None


# ─────────────────── 6DOF NUMERICAL IK TESTS ───────────────────────

def test_6dof_numerical_ik_convergence():
    """
    Verifies the numerical IK solver converges from perturbed initial seeds,
    requiring real iterations to reach tolerance.
    """
    arm = RobotArm6DOF()
    solver = NumericalIKSolver6DOF(arm, pos_tolerance=1e-3, rot_tolerance=1e-2, max_iterations=150)
    test_configs = [
        np.radians([15.0, 30.0, -20.0, 10.0, -35.0, 25.0]),
        np.radians([-20.0, 45.0, -15.0, -10.0, 20.0, -15.0]),
        np.radians([35.0, 20.0, -40.0, 25.0, -20.0, 10.0])
    ]
    rng = np.random.default_rng(42)
    for q_true in test_configs:
        T_target, _ = arm.forward_kinematics(q_true)
        target_pos, target_rot = extract_pose(T_target)
        # Perturb seed by up to ±0.10 rad (~5.7 deg) to verify real multi-iteration convergence
        q_seed = q_true + rng.uniform(-0.10, 0.10, 6)
        success, q_sol, iters, pos_err, rot_err = solver.solve(target_pos, target_rot, q_initial=q_seed)
        assert success, f"IK failed to converge for target {target_pos} (err={pos_err:.4f})"
        assert iters > 0, "Numerical solver should execute at least 1 iteration from perturbed seed"
        assert pos_err < 1e-3, f"Position error {pos_err:.6f} exceeded tolerance"
        assert rot_err < 1e-2, f"Orientation error {rot_err:.6f} exceeded tolerance"
        T_sol, _ = arm.forward_kinematics(q_sol)
        assert np.allclose(T_sol[:3, 3], target_pos, atol=2e-3)
        from inverse_kinematics_6dof import rotation_error_vector
        final_rot_err = np.linalg.norm(rotation_error_vector(T_sol[:3, :3], target_rot))
        assert final_rot_err < 1e-2


def test_6dof_numerical_ik_unreachable_target():
    arm = RobotArm6DOF()
    solver = NumericalIKSolver6DOF(arm, max_iterations=50)
    target_pos = np.array([5.0, 5.0, 5.0])
    target_rot = np.eye(3)
    success, _, iters, pos_err, _ = solver.solve(target_pos, target_rot)
    assert not success, "Solver should report failure on out-of-reach target"
    assert pos_err > 1.0


def test_6dof_joint_limits_respected():
    """Verify joint limits are enforced on a REACHABLE target."""
    arm = RobotArm6DOF()
    solver = NumericalIKSolver6DOF(arm)
    # Use FK of a valid config to guarantee the target is reachable
    q_valid = np.radians([20.0, 25.0, -30.0, 10.0, -20.0, 15.0])
    T_target, _ = arm.forward_kinematics(q_valid)
    target_pos, target_rot = extract_pose(T_target)
    success, q_sol, _, _, _ = solver.solve(target_pos, target_rot, q_initial=q_valid)
    assert success, "Should converge for a reachable target"
    for j in range(6):
        assert arm.joint_limits[j, 0] <= q_sol[j] <= arm.joint_limits[j, 1], f"Joint {j} out of limits"


# ────────────────── 6DOF ANALYTICAL IK TESTS ───────────────────────

def test_6dof_analytical_ik_known_config():
    """Analytical IK must return at least one solution for a known reachable configuration."""
    arm = RobotArm6DOF()
    q_true = np.radians([25.0, 35.0, -30.0, 15.0, -40.0, 20.0])
    T_target, _ = arm.forward_kinematics(q_true)
    target_pos, target_rot = extract_pose(T_target)
    
    solutions = analytical_ik_spherical_wrist(arm, target_pos, target_rot)
    assert len(solutions) > 0, "Analytical IK returned no solutions for a known reachable pose"
    
    for sol in solutions:
        T_check, _ = arm.forward_kinematics(sol)
        assert np.allclose(T_check[:3, 3], target_pos, atol=1e-3), "Position FK mismatch"
        # Orientation check via rotation error
        from inverse_kinematics_6dof import rotation_error_vector
        rot_err = np.linalg.norm(rotation_error_vector(T_check[:3, :3], target_rot))
        assert rot_err < 1e-2, f"Orientation error {rot_err:.4f} exceeds threshold"


def test_6dof_analytical_ik_fk_consistency():
    """Round-trip: FK(q) -> analytical IK -> FK(solution) must reproduce the target."""
    arm = RobotArm6DOF()
    test_configs = [
        np.radians([0.0, 30.0, -20.0, 0.0, -30.0, 0.0]),
        np.radians([45.0, 40.0, -35.0, 10.0, -25.0, 20.0]),
        np.radians([-30.0, 25.0, -40.0, -15.0, 30.0, -10.0]),
    ]
    for q_true in test_configs:
        T_target, _ = arm.forward_kinematics(q_true)
        target_pos, target_rot = extract_pose(T_target)
        solutions = analytical_ik_spherical_wrist(arm, target_pos, target_rot)
        assert len(solutions) > 0, f"No solutions for q={np.degrees(q_true).round(1)}"
        # At least one solution must reproduce the FK pose
        any_match = False
        for sol in solutions:
            T_check, _ = arm.forward_kinematics(sol)
            if np.allclose(T_check[:3, 3], target_pos, atol=1e-3):
                any_match = True
                break
        assert any_match, f"No solution reproduced the target position for q={np.degrees(q_true).round(1)}"


def test_6dof_analytical_ik_two_branches():
    """Analytical IK should find both elbow-up and elbow-down branches when both are within joint limits."""
    arm = RobotArm6DOF()
    # Configuration chosen so that both theta3=45 deg and theta3=180-45=135 deg are within limits [-144 deg, 144 deg]
    q_known = np.radians([0.0, 20.0, 45.0, 0.0, 30.0, 0.0])
    T_target, _ = arm.forward_kinematics(q_known)
    target_pos, target_rot = extract_pose(T_target)
    solutions = analytical_ik_spherical_wrist(arm, target_pos, target_rot)
    
    assert len(solutions) >= 2, f"Expected multiple solutions, found {len(solutions)}"
    
    elbow_angles = [np.degrees(sol[2]) for sol in solutions]
    has_elbow_down = any(np.isclose(a, 45.0, atol=1.0) for a in elbow_angles)
    has_elbow_up = any(np.isclose(a, 135.0, atol=1.0) for a in elbow_angles)
    
    assert has_elbow_down, f"Elbow-down solution (~45 deg) missing. Found elbow angles: {elbow_angles}"
    assert has_elbow_up, f"Elbow-up solution (~135 deg) missing. Found elbow angles: {elbow_angles}"
    
    # Every solution must reproduce position and orientation exactly
    for sol in solutions:
        T_check, _ = arm.forward_kinematics(sol)
        assert np.allclose(T_check[:3, 3], target_pos, atol=1e-4)
        from inverse_kinematics_6dof import rotation_error_vector
        assert np.linalg.norm(rotation_error_vector(T_check[:3, :3], target_rot)) < 1e-4


def test_6dof_analytical_ik_unreachable():
    """Analytical IK on an unreachable target must return an empty list."""
    arm = RobotArm6DOF()
    # 5m away is far outside the ~1.05m total reach
    target_pos = np.array([5.0, 0.0, 0.0])
    target_rot = np.eye(3)
    solutions = analytical_ik_spherical_wrist(arm, target_pos, target_rot)
    assert solutions == [], "Should return empty list for unreachable target"


def test_6dof_analytical_ik_orientation_accuracy():
    """Every accepted solution must match full SE(3) pose within tolerance."""
    from inverse_kinematics_6dof import rotation_error_vector
    arm = RobotArm6DOF()
    q_true = np.radians([15.0, 40.0, -30.0, -20.0, -40.0, 30.0])
    T_target, _ = arm.forward_kinematics(q_true)
    target_pos, target_rot = extract_pose(T_target)
    solutions = analytical_ik_spherical_wrist(arm, target_pos, target_rot)
    assert len(solutions) > 0, "Expected at least one solution"
    for sol in solutions:
        T_check, _ = arm.forward_kinematics(sol)
        pos_err = np.linalg.norm(T_check[:3, 3] - target_pos)
        rot_err = np.linalg.norm(rotation_error_vector(T_check[:3, :3], target_rot))
        assert pos_err < 1e-3, f"Position error {pos_err:.4f} m"
        assert rot_err < 1e-2, f"Orientation error {rot_err:.4f}"


# ───────────────────────── ERROR FUNCTION TESTS ─────────────────────

def test_rotation_error_near_identity():
    """Error vector should be near-zero when R_current == R_target."""
    from inverse_kinematics_6dof import rotation_error_vector
    from rotation_matrices import rot_z
    R = rot_z(0.3)
    err = rotation_error_vector(R, R)
    assert np.linalg.norm(err) < 1e-8


def test_rotation_error_90deg():
    """Error vector magnitude should equal pi/2 for a 90-degree rotation."""
    from inverse_kinematics_6dof import rotation_error_vector
    from rotation_matrices import rot_z
    R_curr = np.eye(3)
    R_tgt = rot_z(np.pi / 2)
    err = rotation_error_vector(R_curr, R_tgt)
    assert abs(np.linalg.norm(err) - np.pi / 2) < 1e-4


def test_rotation_error_180deg():
    """Error vector magnitude should equal pi for a 180-degree rotation (not zero)."""
    from inverse_kinematics_6dof import rotation_error_vector
    from rotation_matrices import rot_z
    R_curr = np.eye(3)
    R_tgt = rot_z(np.pi)
    err = rotation_error_vector(R_curr, R_tgt)
    assert np.linalg.norm(err) > 2.5  # Should be close to pi (~3.14), not near zero


def test_6dof_analytical_ik_wrist_singularity():
    """When theta5 = 0 (axes 4 and 6 collinear), solver must gracefully find valid solution."""
    arm = RobotArm6DOF()
    q_sing = np.radians([20.0, 30.0, -25.0, 15.0, 0.0, 10.0])
    T_target, _ = arm.forward_kinematics(q_sing)
    target_pos, target_rot = extract_pose(T_target)
    solutions = analytical_ik_spherical_wrist(arm, target_pos, target_rot)
    assert len(solutions) > 0, "Failed to resolve pose at wrist singularity (theta5 = 0)"
    # At least one solution must reproduce the position within 1mm
    any_match = False
    for sol in solutions:
        T_check, _ = arm.forward_kinematics(sol)
        if np.allclose(T_check[:3, 3], target_pos, atol=1e-3):
            any_match = True
            break
    assert any_match, "Wrist singularity solution failed position check"


def test_rotation_error_arbitrary_axes_near_180deg():
    """Verify rotation error vector across multiple arbitrary axes at and near 180 degrees."""
    from inverse_kinematics_6dof import rotation_error_vector
    axes = [
        np.array([1.0, -1.0, 0.0]) / np.sqrt(2.0),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
        np.array([1.0, 2.0, 3.0]) / np.linalg.norm([1.0, 2.0, 3.0]),
        np.array([-1.0, 2.0, -0.5]) / np.linalg.norm([-1.0, 2.0, -0.5])
    ]
    for ax in axes:
        for delta in [0.0, 1e-5, 1e-4, 1e-3, 0.05]:
            theta = np.pi - delta
            K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
            R = np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)
            err = rotation_error_vector(np.eye(3), R)
            assert np.isclose(np.linalg.norm(err), theta, atol=1e-4), (
                f"Magnitude error at theta={theta} for axis={ax}: got {np.linalg.norm(err)}"
            )
            if delta > 1e-6:
                computed_axis = err / np.linalg.norm(err)
                assert np.allclose(computed_axis, ax, atol=1e-3)


def test_ik_rejects_invalid_rotation_matrix():
    """Analytical and numerical solvers must raise ValueError if target_rot is not in SO(3)."""
    arm = RobotArm6DOF()
    solver = NumericalIKSolver6DOF(arm)
    target_pos = np.array([0.4, 0.2, 0.3])

    # Non-orthogonal matrix
    R_non_ortho = np.array([[1.0, 0.5, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    with pytest.raises(ValueError):
        analytical_ik_spherical_wrist(arm, target_pos, R_non_ortho)
    with pytest.raises(ValueError):
        solver.solve(target_pos, R_non_ortho)

    # Determinant != 1 (reflection matrix with det = -1)
    R_refl = np.diag([1.0, 1.0, -1.0])
    with pytest.raises(ValueError):
        analytical_ik_spherical_wrist(arm, target_pos, R_refl)
    with pytest.raises(ValueError):
        solver.solve(target_pos, R_refl)

