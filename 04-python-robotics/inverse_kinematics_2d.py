"""
Planar Manipulator Inverse Kinematics (2-Link and 3-Link)
Author: Josiah & Educational Robotics Contributors
License: MIT

This module computes analytical inverse kinematics for 2-link and 3-link planar
robotic arms. It derives both elbow-up and elbow-down configurations, performs
reachability checks, and verifies solutions against forward kinematics.
"""

from typing import Dict, List, Optional, Tuple
import numpy as np
from forward_kinematics_2d import fk_2link_planar, fk_3link_planar


def ik_2link_planar(
    l1: float,
    l2: float,
    x: float,
    y: float
) -> Dict[str, Optional[Tuple[float, float]]]:
    """
    Solve analytical inverse kinematics for a 2-link planar manipulator.
    
    Derivation:
        Distance to target: r^2 = x^2 + y^2
        Law of cosines: r^2 = l1^2 + l2^2 + 2*l1*l2*cos(theta2)
        => cos(theta2) = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2)
        
        If |cos(theta2)| > 1.0, target is outside the workspace.
        
        Two solutions for theta2:
            theta2_down = +arccos(cos_theta2)  (Elbow-Down / Right-arm)
            theta2_up   = -arccos(cos_theta2)  (Elbow-Up / Left-arm)
            
        For theta1:
            theta1 = atan2(y, x) - atan2(l2 * sin(theta2), l1 + l2 * cos(theta2))
            
    Parameters:
        l1, l2: Link lengths in metres.
        x, y: Target coordinates in metres.
        
    Returns:
        Dict with keys 'elbow_up', 'elbow_down', each containing (theta1, theta2) in radians,
        or None if unreachable.
    """
    if l1 <= 0.0 or l2 <= 0.0:
        raise ValueError("Link lengths must be strictly positive.")
        
    r_sq = x**2 + y**2
    r = np.sqrt(r_sq)
    
    # Check reachability bounds: |l1 - l2| <= r <= l1 + l2
    r_max = l1 + l2
    r_min = abs(l1 - l2)
    
    if r > r_max + 1e-9:
        # Target too far
        return {"elbow_up": None, "elbow_down": None}
    if r < r_min - 1e-9:
        # Target in inner workspace hole
        return {"elbow_up": None, "elbow_down": None}
        
    cos_th2 = (r_sq - l1**2 - l2**2) / (2.0 * l1 * l2)
    cos_th2 = float(np.clip(cos_th2, -1.0, 1.0))
    
    # 1. Elbow-Down (theta2 >= 0)
    th2_down = float(np.arccos(cos_th2))
    th1_down = float(np.arctan2(y, x) - np.arctan2(l2 * np.sin(th2_down), l1 + l2 * np.cos(th2_down)))
    
    # 2. Elbow-Up (theta2 <= 0)
    th2_up = -th2_down
    th1_up = float(np.arctan2(y, x) - np.arctan2(l2 * np.sin(th2_up), l1 + l2 * np.cos(th2_up)))
    
    return {
        "elbow_down": (th1_down, th2_down),
        "elbow_up": (th1_up, th2_up)
    }


def ik_3link_planar(
    l1: float,
    l2: float,
    l3: float,
    x: float,
    y: float,
    phi_rad: float
) -> Dict[str, Optional[Tuple[float, float, float]]]:
    """
    Solve analytical inverse kinematics for a 3-link planar manipulator with
    specified target position (x, y) and end-effector orientation phi.
    
    Decoupling strategy:
        The 3rd link ends at (x, y) with orientation phi.
        The wrist position (end of link 2) is:
            x_w = x - l3 * cos(phi)
            y_w = y - l3 * sin(phi)
        Solve 2-link IK for (x_w, y_w) to find (theta1, theta2).
        Then:
            theta3 = phi - theta1 - theta2
    """
    xw = x - l3 * np.cos(phi_rad)
    yw = y - l3 * np.sin(phi_rad)
    
    sols_2link = ik_2link_planar(l1, l2, xw, yw)
    results = {}
    
    for config_name, sol in sols_2link.items():
        if sol is None:
            results[config_name] = None
        else:
            th1, th2 = sol
            th3 = phi_rad - (th1 + th2)
            results[config_name] = (th1, th2, th3)
            
    return results


if __name__ == "__main__":
    print("=" * 60)
    print("Planar Inverse Kinematics Verification")
    print("=" * 60)
    
    l1, l2 = 1.0, 0.8
    target_x, target_y = 1.2, 0.6
    
    print(f"Robot links: l1={l1} m, l2={l2} m")
    print(f"Target pose: ({target_x}, {target_y})")
    
    solutions = ik_2link_planar(l1, l2, target_x, target_y)
    for name, sol in solutions.items():
        if sol is not None:
            th1, th2 = sol
            print(f"\nConfiguration: {name.upper()}")
            print(f"  theta1: {np.rad2deg(th1):.2f} deg, theta2: {np.rad2deg(th2):.2f} deg")
            ee_pose, _ = fk_2link_planar(l1, l2, th1, th2)
            error = np.linalg.norm(ee_pose[:2] - np.array([target_x, target_y]))
            print(f"  FK Re-computed: ({ee_pose[0]:.4f}, {ee_pose[1]:.4f})")
            print(f"  Position Error: {error:.2e} m (PASS: {error < 1e-9})")
            
    # Unreachable target demonstration
    unreachable_sol = ik_2link_planar(l1, l2, 3.0, 3.0)
    print(f"\nTesting unreachable target (3.0, 3.0): {unreachable_sol}")
    print("=" * 60)
