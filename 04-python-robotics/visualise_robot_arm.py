"""
3D Visualization of 6-DOF Robotic Manipulator
Author: Josiah & Educational Robotics Contributors
License: MIT

This script renders an educational 3D wireframe and coordinate frame display
of the 6-DOF robotic manipulator using Matplotlib's mplot3d.
"""

import argparse
from typing import List, Optional
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from forward_kinematics_6dof import RobotArm6DOF
from dh_transform import extract_pose


def plot_coordinate_frame(
    ax: Axes3D,
    origin: np.ndarray,
    rotation: np.ndarray,
    axis_length: float = 0.12,
    linewidth: float = 2.0,
    label_prefix: str = ""
):
    """
    Draw 3D coordinate frame axes using standard RGB convention:
    - X-axis: Red
    - Y-axis: Green
    - Z-axis: Blue
    """
    x_axis = origin + rotation[:, 0] * axis_length
    y_axis = origin + rotation[:, 1] * axis_length
    z_axis = origin + rotation[:, 2] * axis_length
    
    ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]],
            color="red", linewidth=linewidth, label=f"{label_prefix} X" if label_prefix else "")
    ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]],
            color="green", linewidth=linewidth, label=f"{label_prefix} Y" if label_prefix else "")
    ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]],
            color="blue", linewidth=linewidth, label=f"{label_prefix} Z" if label_prefix else "")


def visualize_robot_arm(
    joint_angles: np.ndarray,
    in_degrees: bool = True,
    target_pos: Optional[np.ndarray] = None,
    save_path: Optional[str] = None,
    show_plot: bool = True
) -> plt.Figure:
    """
    Render 3D visualization of the 6-DOF robot arm.
    
    Parameters:
        joint_angles: 6 joint values.
        in_degrees: If True, joint_angles are interpreted as degrees.
        target_pos: Optional target position [x, y, z] to mark on plot.
        save_path: Optional path to save image (e.g. 'robot_pose.png').
        show_plot: Whether to invoke plt.show().
    """
    arm = RobotArm6DOF()
    angles_rad = np.deg2rad(joint_angles) if in_degrees else np.asarray(joint_angles, dtype=float)
    
    T_0_6, transforms = arm.forward_kinematics(angles_rad)
    positions = np.array([T[:3, 3] for T in transforms])
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    
    # 1. Plot Robot Links (Grey/Black lines)
    ax.plot(
        positions[:, 0], positions[:, 1], positions[:, 2],
        "-o", color="#2c3e50", linewidth=4, markersize=8,
        label="Robot Links & Joints"
    )
    
    # 2. Highlight Joint Centers
    ax.scatter(
        positions[:, 0], positions[:, 1], positions[:, 2],
        color="#e74c3c", s=70, zorder=5, label="Joint Centers"
    )
    
    # 3. Plot Base Pedestal (Ground plate)
    ax.plot([0, 0], [0, 0], [-0.05, 0], color="black", linewidth=6)
    ax.scatter([0], [0], [0], color="black", s=120, marker="s", label="Base Origin (Frame 0)")
    
    # 4. Draw Intermediate Coordinate Frames
    for i, T in enumerate(transforms):
        p, R = extract_pose(T)
        scale = 0.15 if i in [0, 6] else 0.08
        prefix = "Base" if i == 0 else ("Flange" if i == 6 else "")
        plot_coordinate_frame(ax, p, R, axis_length=scale, label_prefix=prefix)
        ax.text(p[0] + 0.02, p[1] + 0.02, p[2] + 0.02, f"J{i}", fontsize=9, fontweight="bold")
        
    # 5. Mark Target Position if provided
    if target_pos is not None:
        target_pos = np.asarray(target_pos, dtype=float)
        ax.scatter([target_pos[0]], [target_pos[1]], [target_pos[2]],
                   color="#f39c12", s=150, marker="*", label="Target Pose")
                   
    # Plot configuration and limits
    ax.set_title(
        f"6-DOF Articulated Robot Arm Visualization\n"
        f"q = {np.round(np.degrees(angles_rad), 1)} deg",
        fontsize=12, fontweight="bold"
    )
    ax.set_xlabel("X (metres)")
    ax.set_ylabel("Y (metres)")
    ax.set_zlabel("Z (metres)")
    
    # Set equal aspect ratio
    max_range = 1.0
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([0, max_range * 1.3])
    
    ax.view_init(elev=25, azim=45)
    ax.legend(loc="upper left", bbox_to_anchor=(1.05, 1.0))
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches="tight")
        print(f"Robot visualization saved to: {save_path}")
        
    if show_plot:
        plt.show()
    else:
        plt.close(fig)
        
    return fig


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize 6-DOF Robot Arm Pose")
    parser.add_argument(
        "--angles", nargs=6, type=float, default=[25.0, 35.0, -30.0, 15.0, -40.0, 20.0],
        help="6 joint angles in degrees (default: 25 35 -30 15 -40 20)"
    )
    parser.add_argument("--save", type=str, default="", help="Path to save figure")
    parser.add_argument("--no-show", action="store_true", help="Do not open GUI window")
    args = parser.parse_args()
    
    visualize_robot_arm(
        joint_angles=np.array(args.angles),
        in_degrees=True,
        save_path=args.save if args.save else None,
        show_plot=not args.no_show
    )
