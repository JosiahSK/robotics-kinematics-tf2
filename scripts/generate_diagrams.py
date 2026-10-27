#!/usr/bin/env python3
"""
Generate educational, high-resolution scientific diagrams for robotics textbook.
License: MIT
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Ellipse, Wedge
from matplotlib import cm

# Set high-quality styling
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.size'] = 11
plt.rcParams['axes.labelsize'] = 12
plt.rcParams['axes.titlesize'] = 14
plt.rcParams['xtick.labelsize'] = 10
plt.rcParams['ytick.labelsize'] = 10
plt.rcParams['legend.fontsize'] = 10
plt.rcParams['figure.titlesize'] = 16

repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(repo_root, "04-python-robotics"))

from forward_kinematics_2d import fk_2link_planar
from inverse_kinematics_2d import ik_2link_planar
from forward_kinematics_6dof import RobotArm6DOF
from jacobian import geometric_jacobian_6dof, manipulability_index
from visualise_robot_arm import visualize_robot_arm

output_dir = os.path.join(repo_root, "assets", "generated")
os.makedirs(output_dir, exist_ok=True)


# ─────────────────────────────────────────────────────────────────────────────
# 1. Planar 2R Arm Forward Kinematics
# ─────────────────────────────────────────────────────────────────────────────
def generate_planar_2r_fk():
    l1, l2 = 1.0, 0.5
    th1_deg, th2_deg = 45.0, 45.0
    th1, th2 = np.radians(th1_deg), np.radians(th2_deg)
    
    ee_pose, joints = fk_2link_planar(l1, l2, th1, th2)
    p0 = joints[0]
    p1 = joints[1]
    p2 = ee_pose[:2]
    
    fig, ax = plt.subplots(figsize=(8, 7), dpi=300)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.5)
    
    # Links
    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color='#2980b9', lw=4, solid_capstyle='round', label=r'Link 1 ($l_1 = 1.0\,\mathrm{m}$)')
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='#e67e22', lw=4, solid_capstyle='round', label=r'Link 2 ($l_2 = 0.5\,\mathrm{m}$)')
    
    # Extension of link 1 for relative angle theta2
    p1_ext = p1 + 0.4 * np.array([np.cos(th1), np.sin(th1)])
    ax.plot([p1[0], p1_ext[0]], [p1[1], p1_ext[1]], 'k--', lw=1.5, alpha=0.7)
    
    # Joints
    ax.scatter([p0[0]], [p0[1]], s=140, color='#2c3e50', zorder=5, label=r'Base Joint $J_1$ $(0, 0)$')
    ax.scatter([p1[0]], [p1[1]], s=120, color='#16a085', zorder=5, label=r'Elbow Joint $J_2$')
    ax.scatter([p2[0]], [p2[1]], s=160, marker='*', color='#c0392b', zorder=6, label=f'End-Effector $p_e = ({p2[0]:.3f}, {p2[1]:.3f})$')
    
    # Coordinate axes at base
    ax.annotate('', xy=(0.3, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="red", lw=2))
    ax.annotate('', xy=(0, 0.3), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="green", lw=2))
    ax.text(0.32, 0.02, '$X_0$', color='red', fontweight='bold')
    ax.text(0.02, 0.32, '$Y_0$', color='green', fontweight='bold')
    
    # Angle arcs
    arc1 = Arc((0, 0), 0.5, 0.5, angle=0, theta1=0, theta2=th1_deg, color='#2c3e50', lw=2)
    ax.add_patch(arc1)
    ax.text(0.32 * np.cos(np.radians(22.5)), 0.32 * np.sin(np.radians(22.5)), r'$\theta_1 = 45^\circ$', fontweight='bold', color='#2c3e50')
    
    arc2 = Arc(p1, 0.35, 0.35, angle=0, theta1=th1_deg, theta2=th1_deg + th2_deg, color='#e67e22', lw=2)
    ax.add_patch(arc2)
    ax.text(p1[0] + 0.22 * np.cos(np.radians(67.5)), p1[1] + 0.22 * np.sin(np.radians(67.5)), r'$\theta_2 = 45^\circ$', fontweight='bold', color='#d35400')
    
    # Equations annotation box
    calc_text = (
        r"$\mathbf{Forward\ Kinematics\ Equations:}$" + "\n"
        r"$x_e = l_1 \cos\theta_1 + l_2 \cos(\theta_1 + \theta_2)$" + "\n"
        rf"$x_e = 1.0\cos 45^\circ + 0.5\cos 90^\circ = 0.707 + 0.000 = \mathbf{{{p2[0]:.3f}}}\,\mathrm{{m}}$" + "\n\n"
        r"$y_e = l_1 \sin\theta_1 + l_2 \sin(\theta_1 + \theta_2)$" + "\n"
        rf"$y_e = 1.0\sin 45^\circ + 0.5\sin 90^\circ = 0.707 + 0.500 = \mathbf{{{p2[1]:.3f}}}\,\mathrm{{m}}$" + "\n\n"
        r"$\phi = \theta_1 + \theta_2 = 45^\circ + 45^\circ = \mathbf{90^\circ}$"
    )
    ax.text(
        0.05, 0.95, calc_text, transform=ax.transAxes,
        fontsize=10, verticalalignment='top',
        bbox=dict(boxstyle='round,pad=0.6', facecolor='#f8f9fa', edgecolor='#bdc3c7', alpha=0.95)
    )
    
    ax.set_xlim(-0.2, 1.4)
    ax.set_ylim(-0.2, 1.5)
    ax.set_xlabel('X Position (metres)')
    ax.set_ylabel('Y Position (metres)')
    ax.set_title(r'Planar 2R Robot Arm: Forward Kinematics Geometry & Coordinates', pad=15)
    ax.legend(loc='lower right', framealpha=0.9)
    
    plt.tight_layout()
    path = os.path.join(output_dir, "06-planar-2r-forward-kinematics.png")
    fig.savefig(path, dpi=300)
    plt.close(fig)
    print(f"Generated: {path}")


# ─────────────────────────────────────────────────────────────────────────────
# 2. Planar 2R Arm Inverse Kinematics & Workspace
# ─────────────────────────────────────────────────────────────────────────────
def generate_planar_2r_ik_workspace():
    l1, l2 = 1.0, 0.5
    xt, yt = 1.0, 0.6
    
    sols = ik_2link_planar(l1, l2, xt, yt)
    th1_down, th2_down = sols["elbow_down"]
    th1_up, th2_up = sols["elbow_up"]
    
    _, joints_down = fk_2link_planar(l1, l2, th1_down, th2_down)
    _, joints_up = fk_2link_planar(l1, l2, th1_up, th2_up)
    
    fig, ax = plt.subplots(figsize=(8, 7), dpi=300)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.5)
    
    # Workspace circles
    r_max = l1 + l2
    r_min = abs(l1 - l2)
    outer_circle = plt.Circle((0, 0), r_max, color='#2ecc71', alpha=0.15, label=f'Reachable Workspace ($r \in [{r_min:.1f}, {r_max:.1f}]\,\mathrm{{m}}$)')
    inner_circle = plt.Circle((0, 0), r_min, color='white', zorder=2)
    outer_edge = plt.Circle((0, 0), r_max, color='#27ae60', fill=False, ls='--', lw=2, label=r'Outer Boundary ($l_1 + l_2 = 1.5\,\mathrm{m}$)')
    inner_edge = plt.Circle((0, 0), r_min, color='#e74c3c', fill=False, ls=':', lw=2, label=r'Inner Singularity ($|l_1 - l_2| = 0.5\,\mathrm{m}$)')
    
    ax.add_patch(outer_circle)
    ax.add_patch(inner_circle)
    ax.add_patch(outer_edge)
    ax.add_patch(inner_edge)
    
    # Elbow-Down Solution (Blue)
    ax.plot([0, joints_down[1][0], xt], [0, joints_down[1][1], yt],
            '-o', color='#2980b9', lw=3.5, ms=8, label=rf'Elbow-Down ($\theta_1 = {np.degrees(th1_down):.1f}^\circ, \theta_2 = {np.degrees(th2_down):.1f}^\circ$)')
    
    # Elbow-Up Solution (Orange)
    ax.plot([0, joints_up[1][0], xt], [0, joints_up[1][1], yt],
            '--s', color='#d35400', lw=3.5, ms=8, label=rf'Elbow-Up ($\theta_1 = {np.degrees(th1_up):.1f}^\circ, \theta_2 = {np.degrees(th2_up):.1f}^\circ$)')
    
    # Target marker
    ax.scatter([xt], [yt], s=180, marker='*', color='#c0392b', zorder=10, label=f'Target Pose $p^* = ({xt}, {yt})$')
    
    # Base marker
    ax.scatter([0], [0], s=120, color='black', zorder=10)
    
    # Unreachable target illustration
    ax.scatter([1.7], [0.3], s=100, marker='x', color='#c0392b', zorder=5)
    ax.text(1.72, 0.32, 'Unreachable ($r > 1.5\,\mathrm{m}$)', color='#c0392b', fontsize=9, fontweight='bold')
    
    ax.scatter([0.2], [0.1], s=100, marker='x', color='#c0392b', zorder=5)
    ax.text(0.22, 0.12, 'Inner Hole ($r < 0.5\,\mathrm{m}$)', color='#c0392b', fontsize=9, fontweight='bold')
    
    ax.set_xlim(-0.6, 1.8)
    ax.set_ylim(-0.6, 1.8)
    ax.set_xlabel('X Position (metres)')
    ax.set_ylabel('Y Position (metres)')
    ax.set_title('Inverse Kinematics: Dual Solutions & Reachable Workspace Boundaries', pad=15)
    ax.legend(loc='upper right', framealpha=0.9, fontsize=9)
    
    plt.tight_layout()
    path = os.path.join(output_dir, "07-planar-2r-workspace-ik.png")
    fig.savefig(path, dpi=300)
    plt.close(fig)
    print(f"Generated: {path}")


# ─────────────────────────────────────────────────────────────────────────────
# 3. Manipulator Jacobian & Velocity Ellipsoid
# ─────────────────────────────────────────────────────────────────────────────
def generate_jacobian_ellipsoid():
    arm = RobotArm6DOF()
    
    # Config 1: High manipulability (articulated)
    q_art = np.radians([20.0, 35.0, -30.0, 15.0, -40.0, 20.0])
    J_art = geometric_jacobian_6dof(arm, q_art)
    w_art = manipulability_index(J_art)
    Jv_art = J_art[:2, :3] # 2D translation projection
    
    # Config 2: Near boundary singularity (arm extended)
    q_sing = np.radians([0.0, 0.0, 85.0, 0.0, 5.0, 0.0]) # almost straight
    J_sing = geometric_jacobian_6dof(arm, q_sing)
    w_sing = manipulability_index(J_sing)
    Jv_sing = J_sing[:2, :3]
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5.5), dpi=300)
    
    def plot_ellipse(ax, Jv, title, w, color):
        A = Jv @ Jv.T
        evals, evecs = np.linalg.eigh(A)
        # Sort descending
        idx = np.argsort(evals)[::-1]
        evals = evals[idx]
        evecs = evecs[:, idx]
        
        # Radii of ellipsoid = sqrt(eigenvalues)
        r1, r2 = np.sqrt(np.maximum(0.001, evals))
        angle = np.degrees(np.arctan2(evecs[1, 0], evecs[0, 0]))
        
        ellipse = Ellipse((0, 0), width=2*r1, height=2*r2, angle=angle,
                          facecolor=color, alpha=0.3, edgecolor=color, lw=2.5)
        ax.add_patch(ellipse)
        
        # Draw principal axes
        ax.annotate('', xy=evecs[:, 0]*r1, xytext=(0, 0), arrowprops=dict(arrowstyle="->", color=color, lw=2))
        ax.annotate('', xy=evecs[:, 1]*r2, xytext=(0, 0), arrowprops=dict(arrowstyle="->", color=color, lw=2))
        
        ax.set_aspect('equal')
        ax.grid(True, ls='--', alpha=0.5)
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_xlabel(r'Tip Velocity $\dot{x}$ (m/s)')
        ax.set_ylabel(r'Tip Velocity $\dot{y}$ (m/s)')
        ax.set_title(f"{title}\n" + rf"Yoshikawa Measure $w = \mathbf{{{w:.4f}}}$", fontsize=11, fontweight='bold')
        
    plot_ellipse(ax1, Jv_art, "Articulated Configuration (Dexterous)", w_art, '#2980b9')
    plot_ellipse(ax2, Jv_sing, "Near Singularity (Extended Arm)", w_sing, '#c0392b')
    
    fig.suptitle(r"Manipulator Velocity Ellipsoid: $\mathbf{\dot{x}}^T (\mathbf{J}_v \mathbf{J}_v^T)^{-1} \mathbf{\dot{x}} \leq 1$", fontsize=13, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(output_dir, "10-jacobian-manipulability-ellipsoid.png")
    fig.savefig(path, dpi=300)
    plt.close(fig)
    print(f"Generated: {path}")


# ─────────────────────────────────────────────────────────────────────────────
# 4. 2D Gaussian Distribution & Covariance Ellipses
# ─────────────────────────────────────────────────────────────────────────────
def generate_gaussian_covariance():
    mu = np.array([2.5, 3.0])
    sigma_x, sigma_y = 0.6, 0.3
    rho = 0.7 # correlation
    cov_xy = rho * sigma_x * sigma_y
    Sigma = np.array([
        [sigma_x**2, cov_xy],
        [cov_xy, sigma_y**2]
    ])
    
    # Eigendecomposition
    evals, evecs = np.linalg.eigh(Sigma)
    order = evals.argsort()[::-1]
    evals = evals[order]
    evecs = evecs[:, order]
    angle = np.degrees(np.arctan2(evecs[1, 0], evecs[0, 0]))
    
    # Generate grid for contour
    x = np.linspace(0.5, 4.5, 200)
    y = np.linspace(1.5, 4.5, 200)
    X, Y = np.meshgrid(x, y)
    pos = np.dstack((X, Y))
    
    diff = pos - mu
    Sigma_inv = np.linalg.inv(Sigma)
    exponent = np.einsum('...k,kl,...l->...', diff, Sigma_inv, diff)
    Z = np.exp(-0.5 * exponent) / (2.0 * np.pi * np.sqrt(np.linalg.det(Sigma)))
    
    fig, ax = plt.subplots(figsize=(8, 7), dpi=300)
    ax.set_aspect('equal')
    ax.grid(True, ls='--', alpha=0.5)
    
    # Filled contour
    cs = ax.contourf(X, Y, Z, levels=15, cmap='Blues', alpha=0.85)
    cbar = fig.colorbar(cs, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label('Probability Density Function $p(\mathbf{x})$')
    
    # Draw 1-sigma, 2-sigma, 3-sigma ellipses
    colors = ['#2c3e50', '#2980b9', '#7f8c8d']
    labels = [r'$1\sigma$ (39.3% Conf.)', r'$2\sigma$ (86.5% Conf.)', r'$3\sigma$ (98.9% Conf.)']
    for k, (n_std, col, lbl) in enumerate(zip([1, 2, 3], colors, labels)):
        w, h = 2 * n_std * np.sqrt(evals)
        ell = Ellipse(xy=mu, width=w, height=h, angle=angle,
                      edgecolor=col, facecolor='none', lw=2.2, ls='--' if k > 0 else '-', label=lbl)
        ax.add_patch(ell)
        
    # Draw mean
    ax.scatter([mu[0]], [mu[1]], color='#c0392b', s=120, zorder=5, label=rf'Mean $\mathbf{{\mu}} = [{mu[0]}, {mu[1]}]^T$')
    
    # Draw eigenvector principal axes
    v1 = evecs[:, 0] * np.sqrt(evals[0]) * 2
    v2 = evecs[:, 1] * np.sqrt(evals[1]) * 2
    ax.annotate('', xy=mu + v1, xytext=mu, arrowprops=dict(arrowstyle="->", color="#c0392b", lw=2.5))
    ax.annotate('', xy=mu + v2, xytext=mu, arrowprops=dict(arrowstyle="->", color="#c0392b", lw=2.5))
    ax.text(mu[0] + v1[0]*1.1, mu[1] + v1[1]*1.1, r'$\mathbf{v}_1\sqrt{\lambda_1}$', color='#c0392b', fontweight='bold')
    ax.text(mu[0] + v2[0]*1.1, mu[1] + v2[1]*1.1, r'$\mathbf{v}_2\sqrt{\lambda_2}$', color='#c0392b', fontweight='bold')
    
    ax.set_xlabel('Robot X Position Estimate (m)')
    ax.set_ylabel('Robot Y Position Estimate (m)')
    ax.set_title('Sensor Fusion: 2D Gaussian Uncertainty & Covariance Confidence Ellipses', pad=15)
    ax.legend(loc='upper left', framealpha=0.9, fontsize=9)
    
    plt.tight_layout()
    path = os.path.join(output_dir, "11-gaussian-covariance-ellipse.png")
    fig.savefig(path, dpi=300)
    plt.close(fig)
    print(f"Generated: {path}")


# ─────────────────────────────────────────────────────────────────────────────
# 5. Render Updated 3D Robot Pose Image
# ─────────────────────────────────────────────────────────────────────────────
def generate_robot_pose_image():
    q_demo = np.array([25.0, 35.0, -30.0, 15.0, -40.0, 20.0])
    target_path = os.path.join(repo_root, "04-python-robotics", "robot_pose.png")
    visualize_robot_arm(
        joint_angles=q_demo,
        in_degrees=True,
        save_path=target_path,
        show_plot=False
    )
    print(f"Generated: {target_path}")


if __name__ == "__main__":
    generate_planar_2r_fk()
    generate_planar_2r_ik_workspace()
    generate_jacobian_ellipsoid()
    generate_gaussian_covariance()
    generate_robot_pose_image()
    print("All scientific diagrams successfully generated!")
