# Assessment: Module 53 — TF2 & Coordinate Kinematics Core

**Course:** Robotics & AI Engineering Notes  
**Module:** 53 — Coordinate Frames, TF2 & Perception Geometry  
**Total Marks:** 50 Points  
**Format:** Closed-book, scientific calculator permitted.

---

## Section A: Multiple Choice Questions (10 Points, 2 Points Each)

1. According to ROS REP 105, which coordinate frame is guaranteed to be continuous over time without discrete jumps?
   - A) `map`
   - B) `odom`
   - C) `world`
   - D) `earth`

2. What is the QoS durability setting used for `/tf_static` in ROS 2?
   - A) Volatile
   - B) Best Effort
   - C) Transient Local
   - D) System Default

3. In ROS 2 `geometry_msgs/msg/Quaternion`, how are the 4 components ordered in the message?
   - A) $[w, x, y, z]$
   - B) $[x, y, z, w]$
   - C) $[z, y, x, w]$
   - D) $[w, z, y, x]$

4. If two ROS 2 nodes simultaneously publish transforms with different parents for `base_link`, what error occurs in TF2?
   - A) Frames interpolate smoothly
   - B) The node with highest DDS priority wins
   - C) TF2 violates the single-parent tree rule and drops frames with jitter warnings
   - D) An automatic Kalman filter averages them

5. In the computer vision optical coordinate convention (OpenCV), which axis points forward away from the camera along the optical lens?
   - A) $+X$
   - B) $+Y$
   - C) $+Z$
   - D) $-Z$

---

## Section B: Short Answer & Mathematical Derivations (20 Points)

### Question 6 (5 Points): The Role of `map -> odom`
Explain why autonomous mobile robots running SLAM or AMCL publish corrections to the `map -> odom` transform rather than directly modifying the `odom -> base_link` transform. What would happen to the motor controller if corrections were applied to `odom -> base_link`?

### Question 7 (5 Points): Quaternion Normalisation & Meaning
A team publishes a static transform with quaternion components $q = [0.0, 0.0, 0.0, 0.0]$.
1. Why does TF2 reject this transform?
2. What is the unit quaternion representing zero rotation (Identity)?

### Question 8 (10 Points): 2D Coordinate Transformation
A mobile robot's base frame `base_link` is at $(x = 3.0\,\text{m}, y = 2.0\,\text{m})$ in the `odom` frame, with heading yaw $\theta = 90^\circ$ ($\pi/2\,\text{rad}$).
A laser scanner is mounted on the robot at $x_{\text{rel}} = 0.4\,\text{m}$ forward from `base_link` with zero rotation.
1. Write the $3 \times 3$ homogeneous transformation matrix ${}^{\text{odom}}\mathbf{T}_{\text{base}}$.
2. Write the $3 \times 3$ homogeneous transformation matrix ${}^{\text{base}}\mathbf{T}_{\text{laser}}$.
3. Compute the composite matrix ${}^{\text{odom}}\mathbf{T}_{\text{laser}}$.
4. If the laser detects an obstacle at $[x_{\text{laser}} = 1.0\,\text{m}, y_{\text{laser}} = 0.0\,\text{m}]^T$, find the obstacle's coordinates in the `odom` frame.

---

## Section C: Engineering Practical & Troubleshooting (20 Points)

### Question 9 (10 Points): Multi-Computer Timestamp Drift
An autonomous vehicle has an onboard sensor computer (Computer S) and a GPU planning computer (Computer P).
Computer P repeatedly logs:
`tf2::ExtrapolationException: Lookup would require extrapolation into the future.`
1. Explain the underlying physical and networking cause of this fault.
2. Outline two concrete engineering steps to permanently eliminate this error.

### Question 10 (10 Points): CLI Diagnostics & RViz Fixed Frame
You are tasked with commissioning a new robot. In RViz, you see:
`Global Status: Error - Fixed Frame [map] does not exist.`
Describe the exact sequence of 4 terminal commands you will run to diagnose whether the issue is a missing SLAM node, a broken transform tree, or a topic QoS mismatch.
