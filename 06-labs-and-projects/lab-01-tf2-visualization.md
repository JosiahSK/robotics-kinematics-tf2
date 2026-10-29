# Lab 01: TF2 Tree Inspection and RViz Visualisation

> **Prerequisites:** [01: Coordinate Frames & The TF Tree](../01-module-53-tf2-kinematics/01-coordinate-frames.md), [02: Static & Dynamic Transforms](../01-module-53-tf2-kinematics/02-static-and-dynamic-transforms.md), ROS 2 environment (Jazzy / Humble).  
> **Estimated Time:** 45–60 minutes  
> **Code Files:**
> - Starter / Reference: [Module 53 Lab Reference](../01-module-53-tf2-kinematics/07-ros2-practical-lab.md)
> - Reference RViz Launch: [`eduarm_description/launch/view_arm.launch.py`](../eduarm_description/launch/view_arm.launch.py)

---

## 1. Laboratory Objectives
In this laboratory, you will:
1. Launch a multi-sensor mobile robot transform tree in ROS 2.
2. Inspect the live transform graph using `ros2 run tf2_tools view_frames`.
3. Query real-time transform metrics between `map` and `laser_frame` using `tf2_echo`.
4. Configure RViz2 to visualize coordinate frames with custom axis scales.
5. Diagnose and fix a simulated "disconnected parent" defect.

---

## 2. Theoretical Background
Before executing commands, recall the cardinal rules of TF2 from [Module 53 Chapter 01](../01-module-53-tf2-kinematics/01-coordinate-frames.md):
- A valid TF tree must be a **connected, directed acyclic tree**.
- Every child frame must have **exactly one parent**.
- Dynamic frames publish on `/tf` with Volatile QoS; static frames publish on `/tf_static` with **Transient Local durability QoS**.

---

## 3. Step-by-Step Task Sequence

### Step 3.1: Launch Static Sensor Broadcasters
Open Terminal 1 and broadcast the static LiDAR offset relative to the robot base:
```bash
# Terminal 1: Static LiDAR (25 cm forward, 15 cm up)
ros2 run tf2_ros static_transform_publisher \
  --x 0.25 --y 0.0 --z 0.15 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id laser_frame
```

Open Terminal 2 and broadcast the Camera frame (pitched down by $15^\circ \approx 0.2618\,\text{rad}$):
```bash
# Terminal 2: Static Camera (20 cm forward, 40 cm up, tilted down 15 deg)
ros2 run tf2_ros static_transform_publisher \
  --x 0.20 --y 0.0 --z 0.40 \
  --roll 0.0 --pitch 0.2618 --yaw 0.0 \
  --frame-id base_link --child-frame-id camera_link
```

### Step 3.2: Launch the Odometry Broadcaster
Open Terminal 3 and broadcast dynamic circular odometry:
```bash
# Terminal 3: Dynamic odom -> base_link stream (or custom broadcaster)
ros2 run tf2_ros static_transform_publisher \
  --x 1.0 --y 0.5 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.7854 \
  --frame-id odom --child-frame-id base_link
```

### Step 3.3: Verify Real-Time Transformation
Open Terminal 4 and query the transformation from `odom` to `laser_frame`:
```bash
ros2 run tf2_ros tf2_echo odom laser_frame
```

#### Expected Result:
```text
At time 0.0
- Translation: [1.177, 0.677, 0.150]
- Rotation: in Quaternion [0.000, 0.000, 0.383, 0.924]
- Rotation: in RPY (radian) [0.000, -0.000, 0.785]
- Rotation: in RPY (degree) [0.000, -0.000, 45.000]
```

### Step 3.4: Generate the TF Tree Diagram
In Terminal 4, run:
```bash
ros2 run tf2_tools view_frames
```
This generates `frames.pdf` in your current directory. Verify:
- `odom` is the root node.
- `base_link` has parent `odom`.
- `laser_frame` and `camera_link` both have parent `base_link`.

### Step 3.5: RViz2 Configuration
Launch RViz:
```bash
rviz2
```
1. Set **Global Options $\to$ Fixed Frame** to `odom`.
2. Click **Add** $\to$ select **TF** $\to$ click **OK**.
3. In the TF panel, check **Show Names** and **Show Axes**.
4. Set **Marker Scale** to `0.3`.

---

## 4. Verification Checkpoints
- [ ] `tf2_echo odom laser_frame` prints without timeouts or extrapolation errors.
- [ ] `view_frames` produces a PDF containing all 4 frames in a single tree.
- [ ] In RViz, no frame displays an error status or missing transform warning.

---

## 5. Lab Challenge & Troubleshooting
Simulate a multiple-parent error by running in another terminal:
```bash
ros2 run tf2_ros static_transform_publisher --x 1.0 --y 0.0 --z 0.0 --frame-id world --child-frame-id base_link
```
1. Observe what happens in RViz. Notice that `base_link` jitters between `odom` and `world`.
2. Run `view_frames` and confirm the dual parent conflict.
3. Terminate the conflicting publisher to restore tree integrity.

---

## 6. Extension Task
Add an optical sensor frame `camera_depth_optical_frame` child of `camera_link`:
- In standard computer vision optical frames (REP 103), $+Z$ points forward into the scene, $+X$ points right, and $+Y$ points downward.
- Determine the required roll, pitch, and yaw angles to rotate from standard ROS coordinates ($+X$ forward, $+Y$ left, $+Z$ up) into optical coordinates.
- Broadcast this frame and verify with `ros2 run tf2_ros tf2_echo camera_link camera_depth_optical_frame`.

---

## 7. Lab Deliverables
Submit a brief report including:
1. The generated `frames.pdf` showing your clean, single-parent tree.
2. A screenshot of RViz2 displaying all frames correctly oriented.
3. Your calculated rotation angles for the camera optical extension task.
