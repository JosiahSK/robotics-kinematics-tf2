# URDF Specification for Serial Robot Arms

## 1. Learning Objectives
By completing this chapter, you will be able to:
- Write well-structured Unified Robot Description Format (URDF) XML files.
- Map Denavit–Hartenberg link lengths and offsets directly into URDF `<origin xyz="..." rpy="...">` tags.
- Define revolute joints with realistic hardware limits (`lower`, `upper`, `effort`, `velocity`).
- Attach an end-effector and create a virtual Tool Center Point (`tool_center_point`) fixed joint.

---

## 2. URDF Anatomy: Links vs Joints

In URDF, a robot is modeled as a tree of rigid **`<link>`** elements connected by **`<joint>`** elements:

```mermaid
graph LR
    L1["Parent Link (e.g. link_1)"] -->|"Joint (origin xyz, rpy, axis, limits)"| L2["Child Link (e.g. link_2)"]
```

### The 4 Joint Types in Manipulators:
1. **`revolute`**: Rotates about an axis with hard mechanical limits: `lower` and `upper` (in radians).
2. **`continuous`**: Rotates indefinitely without hard limits (e.g., continuous tool spindles or wheels).
3. **`prismatic`**: Slides linearly along an axis with distance limits in metres.
4. **`fixed`**: Rigid mechanical joint with zero degrees of freedom (used for sensor mounts and TCPs).

---

## 3. Mapping DH Parameters to URDF Origins

In standard DH, transformation $\mathbf{A}_i = \text{Rot}_z(\theta_i) \text{Trans}_z(d_i) \text{Trans}_x(a_i) \text{Rot}_x(\alpha_i)$.
In URDF:
- The joint's rest position and orientation relative to its parent link is specified in the `<origin>` tag:
  ```xml
  <origin xyz="x y z" rpy="roll pitch yaw"/>
  ```
- The axis of motion is defined by `<axis xyz="..."/>`.
  - For rotation about $Z$: `<axis xyz="0 0 1"/>`
  - For rotation about $Y$: `<axis xyz="0 1 0"/>`
  - For rotation about $X$: `<axis xyz="1 0 0"/>`

### Example: Shoulder Joint (Joint 2)
In EduArm-6DOF:
- Waist height: $d_1 = 0.400\,\text{m}$ along $Z$.
- Joint 1 rotates about $Z$:
  ```xml
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.400" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="150.0" velocity="2.5"/>
  </joint>
  ```
- Joint 2 rotates about $Y$ (pitch) with upper arm length $a_2 = 0.500\,\text{m}$:
  ```xml
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="200.0" velocity="2.0"/>
  </joint>
  ```

---

## 4. The Tool Center Point (TCP) Fixed Joint

To track the exact tip of a tool without adding a moving degree of freedom, append a **fixed joint** with a massless link:

```xml
<!-- Rigid mechanical attachment of TCP -->
<joint name="tcp_joint" type="fixed">
  <parent link="tool_flange"/>
  <child link="tool_center_point"/>
  <origin xyz="0.100 0 0" rpy="0 0 0"/>
</joint>

<link name="tool_center_point"/>
```

Now, any ROS 2 node can immediately query the transform `map -> tool_center_point` to know the precise 3D location of the tool tip!

---

## 5. Summary & Next Steps
- URDF provides the XML standard for modeling robot links, visuals, collisions, and joint limits.
- The `robot_state_publisher` reads this URDF to publish the TF tree.
- Now test your knowledge in hands-on practical labs in [**Module 06: Labs and Projects**](../06-labs-and-projects/README.md).
