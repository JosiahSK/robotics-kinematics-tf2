# Robotics Troubleshooting & Common Pitfalls FAQ

This guide compiles the most common bugs, mathematical misconceptions, and software traps encountered when working with kinematics, linear algebra, Python implementations, and ROS 2 TF2.

---

## 1. Mathematical & Trigonometric Pitfalls

### Pitfall 1: `atan2` Argument Reversal
- **The Bug:** Writing `math.atan2(x, y)` instead of `math.atan2(y, x)`.
- **Why it Happens:** In Cartesian coordinate pairs $(x, y)$, $x$ comes first. However, trigonometric tangent is $\tan(\theta) = \frac{y}{x}$, so the standard library function signature is `atan2(y, x)` (numerator first, denominator second).
- **Consequence:** Swapping arguments calculates $\frac{\pi}{2} - \theta$ (or swaps quadrants across the line $y = x$), completely corrupting inverse kinematics and heading angles.
- **Rule of Thumb:** Always write `np.arctan2(delta_y, delta_x)`.

```python
import numpy as np

# CORRECT:
theta = np.arctan2(y, x)

# WRONG: Rotates heading by 90 degrees and flips axis!
theta_wrong = np.arctan2(x, y)
```

---

### Pitfall 2: Degree vs. Radian Confusion
- **The Bug:** Passing degree values directly into `np.sin()`, `np.cos()`, or `rot_z()`.
- **Why it Happens:** Humans intuitively specify angles in degrees ($30^\circ$, $90^\circ$, $180^\circ$), but all standard mathematical libraries, ROS 2 messages, and DH transformation matrices require **radians**.
- **Consequence:** Passing $30^\circ$ into `np.sin(30)` calculates $\sin(30 \text{ rad}) \approx -0.988$ instead of $\sin(30^\circ) = 0.5$.
- **Fix:** Explicitly wrap variables at boundary interfaces:

```python
# Convert user input to radians immediately:
q_rad = np.radians([25.0, 35.0, -30.0, 15.0, -40.0, 20.0])

# Convert back to degrees ONLY for logging or printing:
print(f"Joint angles (deg): {np.degrees(q_rad).round(2)}")
```

---

### Pitfall 3: Matrix Multiplication Order ($\mathbf{A} \mathbf{B} \neq \mathbf{B} \mathbf{A}$)
- **The Bug:** Computing relative transforms using the incorrect multiplication sequence.
- **Why it Happens:** Matrix multiplication is non-commutative.
- **Rules of Spatial Composition:**
  - **Intrinsic (relative to current/local moving frame):** Pre-multiply or post-multiply?
    $$\mathbf{T}_{0}^{2} = \mathbf{T}_{0}^{1} \, \mathbf{T}_{1}^{2} \quad \text{(Post-multiplication by local transform)}$$
  - **Extrinsic (relative to fixed world frame):**
    $$\mathbf{T}_{\text{new}} = \mathbf{T}_{\text{rot}} \, \mathbf{T}_{\text{initial}} \quad \text{(Pre-multiplication by world transform)}$$
- **Verification Checkpoint:** In standard forward kinematics, forward chain evaluation is always:
  $$\mathbf{T}_0^n = \mathbf{T}_0^1 \cdot \mathbf{T}_1^2 \cdot \dots \cdot \mathbf{T}_{n-1}^n$$

---

## 2. Manipulator Kinematics & Modeling Pitfalls

### Pitfall 4: Ignoring Elbow Branches in Inverse Kinematics
- **The Bug:** Taking only the positive square root when calculating $\sin(\theta_2)$ or $\cos(\theta_3)$, assuming only one solution exists.
- **Consequence:** The robot arm is artificially locked into an elbow-up (or elbow-down) posture. If that posture violates joint limits or collides with obstacles, the solver falsely reports "unreachable target".
- **Fix:** Always branch the solution:
  ```python
  # In planar 2-link:
  cos_th2 = (x**2 + y**2 - l1**2 - l2**2) / (2.0 * l1 * l2)
  sin_th2_pos = np.sqrt(max(0.0, 1.0 - cos_th2**2))

  # Branch 1: Elbow Down (th2 >= 0)
  th2_down = np.arctan2(sin_th2_pos, cos_th2)

  # Branch 2: Elbow Up (th2 < 0)
  th2_up = np.arctan2(-sin_th2_pos, cos_th2)
  ```

---

### Pitfall 5: Non-Intersecting Wrist Axes in Pieper Decoupling
- **The Bug:** Applying analytical 3-DOF spherical wrist inverse kinematics to an arm where wrist joint axes do not intersect at a single point.
- **Prerequisite for Analytical IK:** Pieper's criterion requires that three consecutive joint axes intersect at a common point (e.g., joints 4, 5, 6 in a PUMA-like or EduArm-6DOF spherical wrist), OR three axes are parallel.
- **Consequence:** If $d_5 \neq 0$ or $a_4 \neq 0$, the wrist center cannot be isolated purely by subtracting $d_6 \mathbf{R} \hat{\mathbf{z}}$ from the TCP position.
- **Fix:** If the wrist is non-spherical (e.g., UR5/UR10 which have non-zero shoulder/elbow/wrist offsets), use a polynomial solver (subproblem decomposition) or **Numerical Damped Least-Squares (DLS)** IK.

---

### Pitfall 6: Standard DH vs. URDF Frame Discrepancies
- **The Bug:** Expecting the forward kinematics transformation matrix computed from a standard DH table to match URDF link transforms directly.
- **Why it Happens:**
  - Standard DH fixes the $Z_i$ axis strictly along the joint motion axis and defines $X_i$ along the common normal.
  - URDF allows arbitrary link origins and roll-pitch-yaw rotations. Many industrial URDFs define pitch joints with rotation around $+Y$, whereas DH requires joint rotation around $+Z$.
- **Consequence:** At all-zero joint configurations, the end-effector pose will disagree between Python DH code and RViz / URDF.
- **Fix:** Establish an explicit joint angle and frame mapping function (e.g., `dh_to_urdf_joints()` and $\mathbf{R}_{\text{flange}}$ adapter matrix). See [URDF Robot Arm Guide](../../05-ros2-practical/urdf-robot-arm.md#4-standard-dh-frames-vs-urdf-joint-reference-conventions).

---

## 3. Numerical & Jacobian Pitfalls

### Pitfall 7: Divergence Near Singularities in Numerical IK
- **The Bug:** Using the standard Moore–Penrose pseudoinverse $\mathbf{J}^\dagger = \mathbf{J}^T (\mathbf{J} \mathbf{J}^T)^{-1}$ near kinematically singular configurations.
- **Consequence:** Near kinematic singularities (e.g., arm fully outstretched or wrist gimbal lock), $\det(\mathbf{J} \mathbf{J}^T) \to 0$. The pseudoinverse generates massive, near-infinite joint velocity steps ($\Delta \mathbf{q} \to \infty$), causing the solver to oscillate wildly or exit with `NaN`.
- **Fix:** Use Levenberg–Marquardt **Damped Least-Squares (DLS)** evaluated via Singular Value Decomposition (SVD):
  $$\mathbf{J}_{\text{DLS}}^\dagger = \mathbf{V} \operatorname{diag}\left( \frac{\sigma_i}{\sigma_i^2 + \lambda^2} \right) \mathbf{U}^T$$
  where $\lambda \approx 0.01 - 0.05$ prevents denominator collapse.

---

### Pitfall 8: Vanishing Skew Near $180^\circ$ Rotations in $SO(3)$ Log-Map
- **The Bug:** Extracting rotation error vectors solely using the skew-symmetric formula $\frac{\theta}{2 \sin(\theta)} (\mathbf{R} - \mathbf{R}^T)$.
- **Consequence:** When $\theta \to \pi$ ($180^\circ$ flip), $\sin(\theta) \to 0$ while $\mathbf{R} - \mathbf{R}^T \to \mathbf{0}$. The fraction becomes $\frac{0}{0}$ indeterminacy, causing numerical cancellation and returning zero error when the error is actually maximal!
- **Fix:** When $|\theta - \pi| < 10^{-4}$, switch to extracting the unit rotation axis $\mathbf{k}$ from the **symmetric** part of the matrix:
  $$(\mathbf{R} + \mathbf{R}^T) / 2 = \mathbf{I} + \sin^2\left(\frac{\theta}{2}\right) \hat{\mathbf{k}}^2 \implies k_i^2 = \frac{R_{ii} + 1}{2}$$

---

## 4. ROS 2 & TF2 Pitfalls

### Pitfall 9: `ExtrapolationException` When Querying `self.get_clock().now()`
- **The Bug:** Calling `lookup_transform('base_link', 'camera_link', self.get_clock().now())`.
- **Why it Happens:** Network transmission, thread scheduling, and sensor driver cycles mean that the transform stamped with "now" has not yet arrived in the listener's buffer.
- **Consequence:** The listener immediately throws `ExtrapolationException: Lookup would require extrapolation into the future`.
- **Fix:**
  1. Pass `rclpy.time.Time()` (empty time) to query the **most recent available transform**.
  2. If an exact timestamp is required (e.g., transforming a camera image with timestamp $t_{\text{img}}$), supply a small lookup timeout to let the buffer catch up:
     ```python
     transform = tf_buffer.lookup_transform(
         'base_link',
         'camera_link',
         image_msg.header.stamp,
         timeout=rclpy.duration.Duration(seconds=0.05)
     )
     ```

---

### Pitfall 10: Missing Static Transforms Due to Volatile QoS
- **The Bug:** Creating a subscriber to `/tf_static` without setting durability to `Transient Local`.
- **Consequence:** In ROS 2, `/tf_static` is not latched by a central master node. The DDS middleware holds static transform history only for subscribers that request `TRANSIENT_LOCAL` durability. Default subscribers use `VOLATILE` and will miss all static transforms published prior to their startup.
- **Fix:** Always use standard `tf2_ros::Buffer` and `TransformListener`, which automatically configure the correct QoS profile under the hood. If manually inspecting via CLI:
  ```bash
  ros2 topic echo --qos-durability transient_local /tf_static
  ```

---

## 5. Summary Diagnostic Table

| Symptom | Likely Culprit | Quick Verification |
| :--- | :--- | :--- |
| End-effector position mirrors across $Y=X$ line | `atan2(x, y)` swapped | Check `atan2(y, x)` ordering in IK solver |
| Sin/Cos results completely wrong (e.g. $\sin(30) = -0.988$) | Degrees passed instead of radians | Add `np.radians()` conversion |
| Analytical IK reports unreachable when target is inside workspace | Incorrect elbow branch selection | Check both positive and negative roots |
| Numerical IK diverges to `NaN` near zero elevation | Kinematic singularity division by zero | Verify DLS damping factor $\lambda > 0$ |
| Transform listener throws extrapolation error | Querying current clock instead of `Time()` | Use `Time()` or add lookup timeout |
| Late-joining node receives zero `/tf_static` messages | Subscriber QoS durability is Volatile | Set QoS durability to `Transient Local` |
