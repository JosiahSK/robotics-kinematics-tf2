import sys
import os
import pytest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from vectors_matrices import (
    vector_norm,
    normalize_vector,
    dot_product,
    cross_product,
    skew_symmetric_matrix,
    matrix_properties,
    safe_matrix_inverse
)
from dh_transform import (
    create_homogeneous_matrix,
    invert_homogeneous_matrix,
    extract_pose
)
from rotation_matrices import rot_z


def test_vector_operations():
    u = np.array([3.0, 0.0, 0.0])
    v = np.array([0.0, 4.0, 0.0])
    
    assert np.isclose(vector_norm(u), 3.0)
    assert np.allclose(normalize_vector(u), [1.0, 0.0, 0.0])
    assert np.isclose(dot_product(u, v), 0.0)
    
    w = cross_product(u, v)
    assert np.allclose(w, [0.0, 0.0, 12.0])
    
    # Skew-symmetric equivalence: [u]_x * v == u x v
    S_u = skew_symmetric_matrix(u)
    assert np.allclose(S_u @ v, w)


def test_homogeneous_transform_inversion():
    R = rot_z(np.pi / 4)
    p = np.array([1.5, -2.0, 0.8])
    T = create_homogeneous_matrix(R, p)
    
    T_inv = invert_homogeneous_matrix(T)
    assert np.allclose(T @ T_inv, np.eye(4), atol=1e-7)
    assert np.allclose(T_inv @ T, np.eye(4), atol=1e-7)
    
    pos_rec, rot_rec = extract_pose(T)
    assert np.allclose(pos_rec, p)
    assert np.allclose(rot_rec, R)


def test_singular_matrix_inversion_fails():
    singular_M = np.array([
        [1.0, 2.0],
        [2.0, 4.0]
    ])
    with pytest.raises(np.linalg.LinAlgError):
        safe_matrix_inverse(singular_M)
