# Denavit–Hartenberg (DH) Parameters Cheat Sheet

A concise reference guide to the standard Denavit–Hartenberg convention, frame assignment algorithm, canonical parameter tables, and conversion formulas to URDF.

---

## 1. The 4 Standard DH Parameters

Every rigid link in a serial manipulator is uniquely parameterized by 4 geometric quantities:

```
        Z_{i-1}                     Z_i
           |                         |
           |   d_i (along Z_{i-1})   |
           +------------------------>+
           |                         |
           |                         |
  X_{i-1} -+                         +- X_i (along common normal)
            \                       /
             \  θ_i (about Z_{i-1}) \  α_i (about X_i)
```

| Parameter | Name | Axis of Rotation / Measurement | Description | Variable for Revolute? |
| :---: | :--- | :--- | :--- | :---: |
| $\theta_i$ | **Joint Angle** | About $Z_{i-1}$ axis | Angle from $X_{i-1}$ to $X_i$ measured around $Z_{i-1}$ using right-hand rule | **YES ($\theta_i$)** |
| $d_i$ | **Link Offset** | Along $Z_{i-1}$ axis | Distance from origin $O_{i-1}$ to intersection of common normal along $Z_{i-1}$ | NO (Constant) |
| $a_i$ | **Link Length** | Along $X_i$ axis | Length of the common normal: distance from $Z_{i-1}$ to $Z_i$ along $X_i$ | NO (Constant) |
| $\alpha_i$ | **Link Twist** | About $X_i$ axis | Angle from $Z_{i-1}$ to $Z_i$ measured around $X_i$ using right-hand rule | NO (Constant) |

---

## 2. Standard Frame Assignment Algorithm

1. **Assign $Z$-Axes:** Align $Z_{i-1}$ along the axis of motion (rotation or translation) of Joint $i$.
2. **Assign Base Frame (Frame 0):** Choose $O_0$ anywhere on $Z_0$. Choose $X_0$ orthogonal to $Z_0$ in an intuitive direction (typically forward along robot chassis).
3. **Assign Link Frames (Frames 1 to $n-1$):**
   - **Case A (Axes $Z_{i-1}$ and $Z_i$ are non-parallel & non-intersecting):** $X_i$ is collinear with the unique common normal, pointing from $Z_{i-1}$ to $Z_i$.
   - **Case B (Axes $Z_{i-1}$ and $Z_i$ intersect):** Origin $O_i$ is at the intersection point. $X_i$ is perpendicular to the plane formed by $Z_{i-1}$ and $Z_i$ ($X_i = \pm(Z_{i-1} \times Z_i)$). Here $a_i = 0$.
   - **Case C (Axes $Z_{i-1}$ and $Z_i$ are parallel):** The common normal is not unique; pick the one that passes through origin of Joint $i+1$ to make $d_i = 0$.
4. **Complete Right-Handed System:** $Y_i = Z_i \times X_i$.
5. **Assign End-Effector Frame (Frame $n$):** $Z_n$ points along the tool approach vector; $X_n$ is aligned with $X_{n-1}$ when $\theta_n = 0$.

---

## 3. Canonical Model: EduArm-6DOF DH Table

The verified academic model used throughout this textbook:

| Link $i$ | $\theta_i$ | $d_i\,(\text{m})$ | $a_i\,(\text{m})$ | $\alpha_i\,(\text{rad})$ | $\alpha_i\,(^\circ)$ | Joint Range | Mechanical Function |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **1** | $\theta_1$ (var) | $0.400$ | $0.000$ | $+\pi/2$ | $+90^\circ$ | $[-\pi, +\pi]$ | Base Waist (Yaw) |
| **2** | $\theta_2$ (var) | $0.000$ | $0.500$ | $0.000$ | $0^\circ$ | $[-\pi/2, +\pi/2]$ | Shoulder (Pitch) |
| **3** | $\theta_3$ (var) | $0.000$ | $0.000$ | $+\pi/2$ | $+90^\circ$ | $[-0.8\pi, +0.8\pi]$ | Elbow (Pitch) |
| **4** | $\theta_4$ (var) | $0.450$ | $0.000$ | $-\pi/2$ | $-90^\circ$ | $[-\pi, +\pi]$ | Forearm Roll |
| **5** | $\theta_5$ (var) | $0.000$ | $0.000$ | $+\pi/2$ | $+90^\circ$ | $[-\pi/2, +\pi/2]$ | Wrist Pitch |
| **6** | $\theta_6$ (var) | $0.100$ | $0.000$ | $0.000$ | $0^\circ$ | $[-\pi, +\pi]$ | Tool Flange Roll |

---

## 4. Converting Standard DH to ROS 2 URDF

Standard DH tables and CAD/URDF visualization models use different zero reference poses:

| Property | Canonical DH Model | URDF Model (`eduarm_6dof.urdf`) |
| :--- | :--- | :--- |
| **Home Pose ($\mathbf{q} = \mathbf{0}$)** | Forearm hangs along $-Z_0$ (elbow bent $90^\circ$). TCP: $[0.500, 0, -0.150]\,\text{m}$ | Outstretched along $+X_0$. TCP: $[1.050, 0, 0.400]\,\text{m}$ |
| **Pitch Axes** | $Z_1, Z_2$ point along $-Y_0$ | $Y$ axis points along $+Y_0$ |
| **Tool Frame** | $Z$ along tool approach, $X$ up | $X$ along tool approach, $Z$ up |

### Conversion Formulas:
Implemented in [`forward_kinematics_6dof.py`](../../04-python-robotics/forward_kinematics_6dof.py):

$$\begin{aligned}
q_{1, DH} &= q_{1, URDF} \\
q_{2, DH} &= -q_{2, URDF} \\
q_{3, DH} &= \frac{\pi}{2} - q_{3, URDF} \\
q_{4, DH} &= q_{4, URDF} \\
q_{5, DH} &= -q_{5, URDF} \\
q_{6, DH} &= q_{6, URDF}
\end{aligned}$$

$$\mathbf{R}_{URDF} = \mathbf{R}_{DH} \begin{bmatrix} 0 & 0 & 1 \\ 0 & -1 & 0 \\ 1 & 0 & 0 \end{bmatrix}$$
