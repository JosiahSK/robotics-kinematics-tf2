# Notation, Coordinate Frames & SI Units

This guide establishes the mathematical notation, coordinate frame conventions, and physical units used consistently across all modules, Python programs, and ROS 2 practicals.

---

## 1. Coordinate Frame Notation

We adhere to Craig's standard robotics transformation notation:

```
        A                      A
         p                      T
          i                      B
  (point p_i in frame A)  (transform mapping B into A)
```

| Symbol | Mathematical Meaning | Example / Usage |
| :---: | :--- | :--- |
| ${}^A\mathbf{p}$ | 3D position vector of a point expressed in reference frame $\{A\}$. | ${}^{\text{base}}\mathbf{p}_{\text{laser}}$ is the LiDAR's position in base coordinates. |
| ${}^A\mathbf{R}_B$ | $3 \times 3$ rotation matrix describing the orientation of frame $\{B\}$ relative to frame $\{A\}$. | Columns of ${}^A\mathbf{R}_B$ are the unit axes of $\{B\}$ written in $\{A\}$. |
| ${}^A\mathbf{T}_B$ | $4 \times 4$ homogeneous transformation matrix mapping coordinates from $\{B\}$ to $\{A\}$. | ${}^A\mathbf{p} = {}^A\mathbf{T}_B \, {}^B\mathbf{p}$ |
| ${}^B\mathbf{T}_A$ | Inverse transformation mapping $\{A\}$ to $\{B\}$. | ${}^B\mathbf{T}_A = ({}^A\mathbf{T}_B)^{-1}$ |

### Transform Composition Rule:
When chaining transformations, inner frame letters must cancel:
$${}^A\mathbf{T}_C = {}^A\mathbf{T}_B \; {}^B\mathbf{T}_C$$
$${}^0\mathbf{T}_6 = {}^0\mathbf{T}_1 \; {}^1\mathbf{T}_2 \; {}^2\mathbf{T}_3 \; {}^3\mathbf{T}_4 \; {}^4\mathbf{T}_5 \; {}^5\mathbf{T}_6$$

---

## 2. Right-Hand Rule & Axis Conventions

All spatial representations use **right-handed coordinate systems**:

```
        +Z (Thumb)
            |
            |
            +------ +Y (Middle Finger)
           /
          /
        +X (Index Finger)
```

### The RGB = XYZ Visual Standard:
In RViz, Gazebo, and Matplotlib visualizers:
- **Red:** $+X$ axis
- **Green:** $+Y$ axis
- **Blue:** $+Z$ axis

---

## 3. ROS Standards (REP 103 & REP 105)

### Vehicle & Chassis Convention (REP 103):
- **$+X$:** Forward (direction of forward travel)
- **$+Y$:** Left
- **$+Z$:** Up

### Rotation Angles (Euler Z-Y-X / Roll-Pitch-Yaw):
- **Roll ($\phi$):** Counter-clockwise rotation about $+X$ axis.
- **Pitch ($\theta$):** Counter-clockwise rotation about $+Y$ axis.
- **Yaw ($\psi$):** Counter-clockwise rotation about $+Z$ axis.

### Computer Vision vs. Robotics Convention:
- **Robot Body Frame (`camera_link`):** $X$ forward, $Y$ left, $Z$ up.
- **Optical Camera Frame (`camera_color_optical_frame`):** $Z$ forward (viewing ray / depth), $X$ right, $Y$ down.

---

## 4. SI Units Standard

Unless explicitly indicated by the variable name suffix (e.g., `_deg`), **all mathematical calculations, Python functions, and ROS 2 messages must use SI units**:

| Physical Quantity | Preferred Unit | Symbol | Never Use |
| :--- | :--- | :---: | :--- |
| **Length / Distance** | Metres | $\text{m}$ | Centimetres ($\text{cm}$), millimetres ($\text{mm}$), inches |
| **Angle** | Radians | $\text{rad}$ | Degrees ($^\circ$) in internal computations |
| **Time** | Seconds | $\text{s}$ | Milliseconds ($\text{ms}$) in kinematic math |
| **Linear Velocity** | Metres per second | $\text{m/s}$ | $\text{km/h}$, $\text{mph}$ |
| **Angular Velocity** | Radians per second | $\text{rad/s}$ | $\text{RPM}$, $\text{deg/s}$ |
| **Force** | Newtons | $\text{N}$ | Kilograms ($\text{kg}$), pounds ($\text{lbf}$) |
| **Torque** | Newton-metres | $\text{N}\cdot\text{m}$ | $\text{kg}\cdot\text{cm}$, $\text{ft}\cdot\text{lb}$ |
| **Mass** | Kilograms | $\text{kg}$ | Grams ($\text{g}$), pounds ($\text{lbs}$) |
