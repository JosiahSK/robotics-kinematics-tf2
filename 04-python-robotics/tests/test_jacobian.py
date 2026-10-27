import sys
import os
import pytest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from forward_kinematics_6dof import RobotArm6DOF
from jacobian import (
    geometric_jacobian_6dof,
    numerical_jacobian_6dof,
    manipulability_index,
    damped_least_squares_inverse
)


def test_geometric_vs_numerical_jacobian():
    arm = RobotArm6DOF()
    test_angles = [
        np.zeros(6),
        np.radians([15.0, 30.0, -20.0, 10.0, -35.0, 25.0]),
        np.radians([-45.0, 20.0, 15.0, -30.0, 45.0, -10.0])
    ]
    
    for q in test_angles:
        J_geom = geometric_jacobian_6dof(arm, q)
        J_num = numerical_jacobian_6dof(arm, q, delta_q=1e-6)
        
        # Linear velocity rows (first 3 rows) should match within tight tolerance
        assert np.allclose(J_geom[:3, :], J_num[:3, :], atol=1e-4)
        # Angular velocity rows (last 3 rows) should also match
        assert np.allclose(J_geom[3:, :], J_num[3:, :], atol=1e-3)


def test_dls_inverse_stability_at_singularity():
    arm = RobotArm6DOF()
    # At full extension, arm is singular
    q_singular = np.zeros(6)
    J = geometric_jacobian_6dof(arm, q_singular)
    
    # Standard pseudo-inverse might blow up, but DLS should remain well-bounded
    damping = 0.1
    J_dls = damped_least_squares_inverse(J, damping=damping)
    
    assert np.all(np.isfinite(J_dls))
    assert np.max(np.abs(J_dls)) < 100.0


def test_manipulability_measure():
    arm = RobotArm6DOF()
    q_regular = np.radians([20.0, 40.0, -30.0, 20.0, -40.0, 10.0])
    J_reg = geometric_jacobian_6dof(arm, q_regular)
    w_reg = manipulability_index(J_reg)
    
    assert w_reg > 1e-4
    assert np.isfinite(w_reg)


def test_dls_uses_svd_robustness():
    """DLS inverse via SVD should handle near-singular Jacobians without NaN/Inf."""
    from jacobian import damped_least_squares_inverse
    # Create a rank-deficient Jacobian (singular case)
    J = np.zeros((6, 6))
    J[0, 0] = 1.0  # Only one non-zero singular value
    J_dls = damped_least_squares_inverse(J, damping=0.05)
    assert np.all(np.isfinite(J_dls)), "DLS should return finite values for near-singular Jacobian"
    assert J_dls.shape == (6, 6)
