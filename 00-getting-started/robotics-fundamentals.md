# Robotics Fundamentals: Architecture, Frames, and Degrees of Freedom

## 1. Learning Objectives
By completing this chapter, you will be able to:
- Formulate the core loop of an autonomous robotic system (Sense $\to$ Plan $\to$ Act).
- Compute the mobility and Degrees of Freedom (DoF) of open kinematic chains using Grübler–Kutzbach criterion.
- Differentiate between revolute ($R$) and prismatic ($P$) joints.
- Apply standard right-handed coordinate frame conventions (ROS REP 103).
- Understand why transformations are central to robotics software engineering.

---

## 2. Intuitive Explanation
A robot is not merely a computer, and not merely a mechanical mechanism. A robot is an intelligent physical agent operating in three-dimensional space. To perform any task—such as picking up an apple, vacuuming a carpet, or welding an automotive chassis—the robot must constantly solve two questions:
1. **"Where am I and where is the target relative to me?"** (Localization & Perception)
2. **"How must I move my actuators to reach the target safely?"** (Kinematics & Control)

To answer these questions, every link, sensor, camera, wheel, and gripper on the robot must have its own coordinate frame. Robotics is the art of translating descriptions between these frames.

```mermaid
graph LR
    Sensors["Sensors (LiDAR, Camera, Encoders)"] -->|Perception| StateEst["State Estimation (TF2 / EKF)"]
    StateEst -->|Robot Pose & Map| Planner["Planning (Kinematics / Pathfinding)"]
    Planner -->|Joint Commands| Actuators["Actuators (Motors, Hydraulics)"]
    Actuators -->|Physical Motion| Environment["Real World"]
    Environment --> Sensors
```

---

## 3. Mathematical Formulation: Mobility & Degrees of Freedom

The **Degrees of Freedom (DoF)** of a mechanical system is the minimum number of independent coordinates required to completely specify its configuration in space.

### 3.1 Grübler–Kutzbach Criterion
For a mechanism moving in $m$-dimensional space ($m=3$ for planar systems, $m=6$ for spatial 3D mechanisms):

$$M = m(N - 1 - j) + \sum_{i=1}^{j} f_i$$

### 4. Symbol Definitions
- $M$: Mobility (total Degrees of Freedom of the mechanism).
- $m$: Dimensionality of operational space ($m=3$ for planar, $m=6$ for 3D space).
- $N$: Total number of rigid links (including the fixed base link as Link 0).
- $j$: Total number of joints connecting the links.
- $f_i$: Degrees of freedom permitted by joint $i$.
  - Revolute joint ($R$): $f_i = 1$ (1 rotation about joint axis).
  - Prismatic joint ($P$): $f_i = 1$ (1 translation along joint axis).
  - Spherical joint ($S$): $f_i = 3$ (3 rotational freedoms).

For a typical single-chain serial spatial manipulator with $n$ joints, where each joint has 1 DoF ($f_i = 1$), the number of joints is $j = n$, and the total number of links is $N = n + 1$ (base + $n$ moving links). Substituting into the spatial formula ($m=6$):

$$M = 6((n+1) - 1 - n) + \sum_{i=1}^{n} 1 = 6(0) + n = n$$

Thus, an open spatial serial arm with $n$ 1-DoF joints possesses exactly $n$ degrees of freedom.

---

## 5. Step-by-Step Derivation: Planar Four-Bar Linkage vs Serial Arm

Let us verify Grübler's equation on a closed-loop planar four-bar mechanism ($m=3$):
1. **Count links:** 4 links (Link 0 is the ground frame, Links 1, 2, 3 move). So $N = 4$.
2. **Count joints:** 4 1-DoF revolute pin joints. So $j = 4$, with $f_1 = f_2 = f_3 = f_4 = 1$.
3. **Substitute into formula:**
   $$M = 3(4 - 1 - 4) + (1 + 1 + 1 + 1) = 3(-1) + 4 = 1$$
4. **Interpretation:** Although there are 4 links and 4 joints, the closed mechanical loop constrains motion such that selecting a single joint angle $\theta_1$ uniquely determines the positions of all other links.

In contrast, for a **serial arm**, each joint can move independently without closed mechanical constraints.

---

## 6. Fully Worked Numerical Example: 6-DoF Industrial Arm

Consider an industrial welding arm with a fixed pedestal and 6 revolute joints.
- $m = 6$ (Spatial 3D mechanism)
- $N = 7$ links (Base link 0 + 6 articulated arm links)
- $j = 6$ revolute joints, each with $f_i = 1$
- Calculate mobility $M$:
  $$M = 6(7 - 1 - 6) + (1 + 1 + 1 + 1 + 1 + 1) = 6(0) + 6 = 6\text{ DoF}$$
- **Physical Meaning:** A 6-DoF robot in 3D space can position its tool tip at any arbitrary $(x, y, z)$ Cartesian coordinate and orient it with any arbitrary $(\text{roll}, \text{pitch}, \text{yaw})$ within its reachable workspace.

---

## 7. Coordinate Frame Standards (ROS REP 103)

In robotics software (and ROS 2 in particular), we follow **REP 103** (Standard Units of Measure & Coordinate Conventions):
- **Right-Handed System:** If you point the index finger of your right hand along $+X$, and curl your middle finger toward $+Y$, your thumb points along $+Z$.
- **Axis orientations:**
  - $+X$: Forward.
  - $+Y$: Left.
  - $+Z$: Up.
- **Rotation signs:** Positive rotation about an axis follows the right-hand grip rule (counter-clockwise when looking down the positive axis toward the origin).

```mermaid
graph TD
    Z["+Z (Up / Blue)"]
    X["+X (Forward / Red)"]
    Y["+Y (Left / Green)"]
```

---

## 8. Python Demonstration

Students can verify Grübler's mobility formula across various kinematic structures using this clean function:

```python
def gruebler_mobility(num_links: int, joint_dofs: list[int], spatial: bool = True) -> int:
    """Compute mechanism mobility via Gruebler-Kutzbach criterion."""
    m = 6 if spatial else 3
    num_joints = len(joint_dofs)
    dof_sum = sum(joint_dofs)
    return m * (num_links - 1 - num_joints) + dof_sum

# Test 6-DoF Serial Manipulator:
print("6-DoF Serial Arm:", gruebler_mobility(num_links=7, joint_dofs=[1]*6, spatial=True))
# Test Planar 4-bar linkage:
print("Planar 4-Bar:", gruebler_mobility(num_links=4, joint_dofs=[1]*4, spatial=False))
```

---

## 9. Common Mistakes & Debugging Tips
1. **Forgetting Link 0 (Ground/Base):** Beginners often count only moving links ($N=6$). Link 0 (the fixed base) must always be counted ($N=7$).
2. **Left-Handed vs Right-Handed Frames:** Confusing $+Y$ right with $+Y$ left. In standard robotics (ROS / ISO), $+Y$ is **LEFT**, not right.
3. **Unit Inconsistencies:** Mixing degrees and radians or inches and millimetres. Always convert immediately to SI units (metres and radians) at the hardware driver interface.

---

## 10. Exercises

### Beginner
1. A SCARA robot has 3 revolute joints and 1 prismatic joint in 3D space. Calculate the total degrees of freedom and list the number of links.
2. In ROS REP 103, which color in RViz standardly corresponds to the $X$, $Y$, and $Z$ axes?

### Intermediate
3. Explain why a 5-DoF robot arm cannot achieve an arbitrary orientation at an arbitrary position in 3D space.
4. Calculate the mobility of a planar two-link pendulum pinned to the ceiling.

### Advanced
5. Derive Grübler's formula for a parallel robot (e.g. a Delta robot or Stewart platform) with 6 identical kinematic legs connecting a base platform to a mobile platform.

---

## 11. Summary & Next Steps
- A serial robot's degrees of freedom match the number of independent 1-DoF joints in its chain.
- Robotics systems rely strictly on right-handed coordinate frames with $+X$ forward, $+Y$ left, and $+Z$ up.
- Next, review [**Mathematical Prerequisites**](mathematical-prerequisites.md) to brush up on vectors, trigonometry, and calculus before exploring [**Module 53: TF2 & Kinematics Core**](../01-module-53-tf2-kinematics/README.md).
