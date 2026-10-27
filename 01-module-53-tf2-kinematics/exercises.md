# Module 53: Exercises — TF2 & Kinematics Core

Test your comprehension across beginner, intermediate, and advanced problems. Solutions are provided in [`solutions.md`](solutions.md).

---

## Beginner Level

### Exercise 1.1: Standard Mobile Robot Hierarchy
State the standard coordinate frame hierarchy for an autonomous mobile robot equipped with a base chassis, wheel odometry, a map, and a front LiDAR sensor according to ROS REP 105. Write the parent-child relationships.

### Exercise 1.2: Quaternion Identification
Given the unit quaternion $\mathbf{q} = [0.0, 0.0, 0.0, 1.0]$:
1. Identify the scalar and vector components according to ROS 2 convention.
2. What physical 3D rotation does this quaternion represent?

### Exercise 1.3: Static vs Dynamic Transforms
A robot has:
- A front bumper radar rigidly bolted with 4 screws.
- A motorized steerable wheel pivot.
For each item, state whether its transform should be published to `/tf` or `/tf_static`, and justify your choice based on network bandwidth and Quality of Service (QoS).

---

## Intermediate Level

### Exercise 1.4: 2D Transformation Matrix Composition
A mobile robot's base frame `base_link` is located at $(x=2.0\,\text{m}, y=1.0\,\text{m})$ in the `odom` frame, facing with heading angle $\theta = 90^\circ$ ($\pi/2\,\text{rad}$).
A 2D LiDAR is mounted on the robot at $x_{\text{rel}} = 0.5\,\text{m}$ forward from the robot center with no rotation relative to `base_link`.
1. Write the $3 \times 3$ homogeneous transformation matrix ${}^{\text{odom}}\mathbf{T}_{\text{base}}$.
2. Write the $3 \times 3$ homogeneous transformation matrix ${}^{\text{base}}\mathbf{T}_{\text{laser}}$.
3. Compute the composite transformation matrix ${}^{\text{odom}}\mathbf{T}_{\text{laser}}$.
4. If the laser detects a reflective marker at $(x_{\text{laser}} = 1.0\,\text{m}, y_{\text{laser}} = 0.0\,\text{m})$ in its own sensor frame, what are the coordinates of the marker in the `odom` frame?

### Exercise 1.5: SLAM Correction Computation
An AMCL localization node detects that the robot's true pose in the `map` frame is:
$${}^{\text{map}}\mathbf{p}_{\text{base}} = [5.0, 3.0]^T\,\text{m}, \quad \theta_{\text{map}} = 0^\circ$$
Meanwhile, the wheel odometry node reports the robot's pose in the `odom` frame as:
$${}^{\text{odom}}\mathbf{p}_{\text{base}} = [5.2, 2.7]^T\,\text{m}, \quad \theta_{\text{odom}} = 0^\circ$$
Calculate the required translation vector $[x, y]^T$ of the transform ${}^{\text{map}}\mathbf{T}_{\text{odom}}$ that the AMCL node must publish on `/tf` to reconcile this difference.

---

## Advanced Level

### Exercise 1.6: Camera Optical to Base Frame 3D Derivation
An RGB-D camera is mounted on a robot mast:
- Translation: $0.10\,\text{m}$ along $+X$, $0.00\,\text{m}$ along $+Y$, $0.80\,\text{m}$ along $+Z$ relative to `base_link`.
- Tilt: The camera is tilted downward by $30^\circ$ (rotation of $+30^\circ$ around the robot's pitch $Y$-axis).
- The camera outputs point clouds in `camera_depth_optical_frame` (where $+Z_{\text{opt}}$ is forward along the lens, $+X_{\text{opt}}$ is right, and $+Y_{\text{opt}}$ is down).

1. Derive the full $4 \times 4$ transformation matrix ${}^{\text{base}}\mathbf{T}_{\text{opt}}$.
2. If an obstacle point is detected at ${}^{\text{opt}}\mathbf{p} = [0.20, 0.10, 1.50]^T\,\text{m}$, calculate its exact $(x, y, z)$ position in `base_link`.

### Exercise 1.7: Diagnosing a Multi-Machine TF Desynchronization
A team deploys an autonomous rover with two onboard computers:
- Computer A runs the motor drivers, IMU, and broadcasts `odom -> base_link` at 50 Hz.
- Computer B runs the LiDAR, SLAM, and autonomous path planner.
The path planner on Computer B repeatedly throws `tf2::ExtrapolationException: Lookup would require extrapolation into the future`.
Explain the physical and network cause of this error, and outline two engineering solutions to eliminate it.
