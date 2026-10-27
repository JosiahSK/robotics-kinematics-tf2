import sys
import os
import pytest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from dh_transform import dh_matrix, chain_dh_matrices, DHParameter, extract_pose


def test_standard_dh_formula():
    """
    Verify the standard DH matrix matches the analytical decomposition:
    A_i = Rot(z, theta) * Trans(z, d) * Trans(x, a) * Rot(x, alpha)
    """
    theta = 0.4
    d = 0.35
    a = 0.25
    alpha = -0.5
    
    A_implemented = dh_matrix(theta, d, a, alpha)
    
    # Analytical derivation
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    A_expected = np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0]
    ])
    
    assert np.allclose(A_implemented, A_expected, atol=1e-12)


def test_dh_degrees_flag():
    A_rad = dh_matrix(theta=np.pi / 2, d=0.5, a=0.2, alpha=np.pi / 2, in_degrees=False)
    A_deg = dh_matrix(theta=90.0, d=0.5, a=0.2, alpha=90.0, in_degrees=True)
    assert np.allclose(A_rad, A_deg, atol=1e-12)


def test_dh_chaining():
    dh_table = [
        DHParameter(theta=np.pi / 2, d=0.2, a=0.0, alpha=np.pi / 2),
        DHParameter(theta=0.0, d=0.0, a=0.5, alpha=0.0)
    ]
    T_total, intermediate = chain_dh_matrices(dh_table)
    
    assert len(intermediate) == 3  # [T0, T1, T2]
    assert np.allclose(intermediate[0], np.eye(4))
    assert np.allclose(intermediate[-1], T_total)
    
    pos, rot = extract_pose(T_total)
    assert np.isclose(np.linalg.det(rot), 1.0, atol=1e-7)
