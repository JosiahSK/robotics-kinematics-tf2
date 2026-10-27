# Module 53: Solutions — TF2 & Kinematics Core

Detailed, step-by-step mathematical and software solutions for [Module 53 Exercises](exercises.md).

---

## Solution 1.1: Standard Mobile Robot Hierarchy
According to **ROS REP 105**, the required parent $\to$ child relationships are:
1. `map` $\to$ `odom` (Published by SLAM / AMCL localization nodes; absorbs drift).
2. `odom` $\to$ `base_link` (Published by wheel odometry / EKF state estimation; continuous and smooth).
3. `base_link` $\to$ `laser_frame` (Published by `robot_state_publisher` or `static_transform_publisher`).

---

## Solution 1.2: Quaternion Identification
1. In ROS 2 (`geometry_msgs/msg/Quaternion`), ordering is $[x, y, z, w]$.
   - Vector imaginary components: $x = 0.0$, $y = 0.0$, $z = 0.0$.
   - Scalar real component: $w = 1.0$.
2. This represents the **Identity Rotation** ($0^\circ$ rotation about any axis). Frame orientation is perfectly aligned with the reference frame.

---

## Solution 1.3: Static vs Dynamic Transforms
- **Front Bumper Radar:** Publish to **`/tf_static`**.
  - *Reasoning:* The radar is rigidly bolted and does not move relative to `base_link`. Publishing once via Transient Local QoS avoids wasting network bandwidth and CPU cycles.
- **Motorized Steerable Wheel Pivot:** Publish to **`/tf`**.
  - *Reasoning:* The pivot angle changes continuously during driving. High-frequency streaming (50–100 Hz) with Volatile QoS is required so controllers know the real-time wheel orientation.

---

## Solution 1.4: 2D Transformation Matrix Composition

### 1. Form ${}^{\text{odom}}\mathbf{T}_{\text{base}}$:
Robot at $(x=2.0, y=1.0)$, heading $\theta = 90^\circ$:
$${}^{\text{odom}}\mathbf{T}_{\text{base}} = \begin{bmatrix} \cos(90^\circ) & -\sin(90^\circ) & 2.0 \\ \sin(90^\circ) & \cos(90^\circ) & 1.0 \\ 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 0 & -1 & 2.0 \\ 1 & 0 & 1.0 \\ 0 & 0 & 1 \end{bmatrix}$$

### 2. Form ${}^{\text{base}}\mathbf{T}_{\text{laser}}$:
Laser at $(x=0.5, y=0.0)$, rotation $\theta = 0^\circ$:
$${}^{\text{base}}\mathbf{T}_{\text{laser}} = \begin{bmatrix} 1 & 0 & 0.5 \\ 0 & 1 & 0.0 \\ 0 & 0 & 1 \end{bmatrix}$$

### 3. Compute ${}^{\text{odom}}\mathbf{T}_{\text{laser}}$:
$${}^{\text{odom}}\mathbf{T}_{\text{laser}} = {}^{\text{odom}}\mathbf{T}_{\text{base}} \; {}^{\text{base}}\mathbf{T}_{\text{laser}} = \begin{bmatrix} 0 & -1 & 2.0 \\ 1 & 0 & 1.0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 1 & 0 & 0.5 \\ 0 & 1 & 0.0 \\ 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 0 & -1 & 2.0 \\ 1 & 0 & 1.5 \\ 0 & 0 & 1 \end{bmatrix}$$

### 4. Transform Marker Point:
${}^{\text{laser}}\mathbf{p} = [1.0, 0.0, 1]^T$:
$${}^{\text{odom}}\mathbf{p} = \begin{bmatrix} 0 & -1 & 2.0 \\ 1 & 0 & 1.5 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 1.0 \\ 0.0 \\ 1 \end{bmatrix} = \begin{bmatrix} 0(1.0) - 1(0.0) + 2.0 \\ 1(1.0) + 0(0.0) + 1.5 \\ 1 \end{bmatrix} = \begin{bmatrix} 2.0 \\ 2.5 \\ 1 \end{bmatrix}$$
**Answer:** The marker is at $(x = 2.0\,\text{m}, y = 2.5\,\text{m})$ in the `odom` frame.

---

## Solution 1.5: SLAM Correction Computation
The relationship between frames is:
$${}^{\text{map}}\mathbf{T}_{\text{base}} = {}^{\text{map}}\mathbf{T}_{\text{odom}} \; {}^{\text{odom}}\mathbf{T}_{\text{base}}$$
Inverting ${}^{\text{odom}}\mathbf{T}_{\text{base}}$:
$${}^{\text{map}}\mathbf{T}_{\text{odom}} = {}^{\text{map}}\mathbf{T}_{\text{base}} \left({}^{\text{odom}}\mathbf{T}_{\text{base}}\right)^{-1}$$
Since $\theta_{\text{map}} = \theta_{\text{odom}} = 0^\circ$, rotations are identity.
Translation vectors subtract directly:
$$\mathbf{p}_{\text{map\_to\_odom}} = \mathbf{p}_{\text{map}} - \mathbf{p}_{\text{odom}} = \begin{bmatrix} 5.0 \\ 3.0 \end{bmatrix} - \begin{bmatrix} 5.2 \\ 2.7 \end{bmatrix} = \begin{bmatrix} -0.2 \\ +0.3 \end{bmatrix}\,\text{m}$$
**Answer:** The AMCL node must broadcast ${}^{\text{map}}\mathbf{T}_{\text{odom}}$ with translation $x = -0.20\,\text{m}$ and $y = +0.30\,\text{m}$.

---

## Solution 1.6: Camera Optical to Base Frame 3D Derivation

### 1. Derive ${}^{\text{base}}\mathbf{T}_{\text{opt}}$:
- **Pitch rotation by $+30^\circ$ about $Y$:**
  $$\mathbf{R}_y(30^\circ) = \begin{bmatrix} \cos(30^\circ) & 0 & \sin(30^\circ) \\ 0 & 1 & 0 \\ -\sin(30^\circ) & 0 & \cos(30^\circ) \end{bmatrix} = \begin{bmatrix} 0.8660 & 0 & 0.5000 \\ 0 & 1 & 0 \\ -0.5000 & 0 & 0.8660 \end{bmatrix}$$
- **Optical convention rotation:**
  $$\mathbf{R}_{\text{opt}} = \begin{bmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix}$$
- **Total rotation ${}^{\text{base}}\mathbf{R}_{\text{opt}} = \mathbf{R}_y(30^\circ) \; \mathbf{R}_{\text{opt}}$:**
  $${}^{\text{base}}\mathbf{R}_{\text{opt}} = \begin{bmatrix} 0.8660 & 0 & 0.5000 \\ 0 & 1 & 0 \\ -0.5000 & 0 & 0.8660 \end{bmatrix} \begin{bmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix} = \begin{bmatrix} 0 & -0.5000 & 0.8660 \\ -1 & 0 & 0 \\ 0 & -0.8660 & -0.5000 \end{bmatrix}$$
- **Total $4 \times 4$ Transform:**
  $${}^{\text{base}}\mathbf{T}_{\text{opt}} = \begin{bmatrix} 0 & -0.5000 & 0.8660 & 0.10 \\ -1 & 0 & 0 & 0.00 \\ 0 & -0.8660 & -0.5000 & 0.80 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### 2. Transform the detected point ${}^{\text{opt}}\mathbf{p} = [0.20, 0.10, 1.50]^T$:
$${}^{\text{base}}\mathbf{p} = \begin{bmatrix} 0(0.20) - 0.5000(0.10) + 0.8660(1.50) + 0.10 \\ -1(0.20) + 0(0.10) + 0(1.50) + 0.00 \\ 0(0.20) - 0.8660(0.10) - 0.5000(1.50) + 0.80 \end{bmatrix} = \begin{bmatrix} -0.05 + 1.2990 + 0.10 \\ -0.20 \\ -0.0866 - 0.7500 + 0.80 \end{bmatrix} = \begin{bmatrix} 1.349\,\text{m} \\ -0.200\,\text{m} \\ -0.037\,\text{m} \end{bmatrix}$$
**Answer:** The obstacle is located at $x = 1.349\,\text{m}$ (ahead), $y = -0.200\,\text{m}$ (right), and $z = -0.037\,\text{m}$ (just below base origin).

---

## Solution 1.7: Diagnosing Multi-Machine TF Desynchronization
- **Cause:** The system clocks on Computer A and Computer B are not synchronized. If Computer B's clock runs ahead of Computer A's clock by even $15\,\text{ms}$, Computer B requests a transform at its current local time $t_{\text{query}}$, which Computer A has not yet published. TF2 interprets this as requesting a transform in the future.
- **Engineering Solutions:**
  1. Configure **`chrony`** or **`NTP`** with Computer A acting as an NTP master server and Computer B synchronized over the local ethernet switch.
  2. In ROS 2 subscriber callbacks, do not query TF using `node.get_clock().now()`. Instead, query using the message header's timestamp `msg.header.stamp`, and provide a generous lookup timeout: `timeout=Duration(seconds=0.1)`.
