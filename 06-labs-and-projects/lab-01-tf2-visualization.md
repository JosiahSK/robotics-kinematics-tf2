# Lab 01: TF2 Tree Inspection and RViz Visualisation

## 1. Laboratory Objectives
In this laboratory, you will:
1. Launch a multi-sensor mobile robot transform tree in ROS 2.
2. Inspect the live transform graph using `ros2 run tf2_tools view_frames`.
3. Query real-time transform metrics between `map` and `laser_frame` using `tf2_echo`.
4. Configure RViz2 to visualize coordinate frames with custom axis scales.
5. Diagnose and fix a simulated "disconnected parent" defect.

---

## 2. Theoretical Background
Before writing code, recall the cardinal rule of TF2 from [Module 53 Chapter 01](../01-module-53-tf2-kinematics/01-coordinate-frames.md):
- A valid TF tree must be a **connected, directed acyclic tree**.
- Every child frame must have **exactly one parent**.

---

## 3. Step-by-Step Lab Procedure

### Step 3.1: Launch Static Sensor Broadcasters
Open Terminal 1 and broadcast the static LiDAR and Camera offsets:
```bash
# Terminal 1: Static LiDAR (25 cm forward, 15 cm up)
ros2 run tf2_ros static_transform_publisher --x 0.25 --y 0.0 --z 0.15 --roll 0.0 --pitch 0.0 --yaw 0.0 --frame-id base_link --child-frame-id laser_frame
```

Open Terminal 2 and broadcast the Camera frame:
```bash
# Terminal 2: Static Camera (20 cm forward, 40 cm up, tilted down 15 deg)
ros2 run tf2_ros static_transform_publisher --x 0.20 --y 0.0 --z 0.40 --roll 0.0 --pitch 0.2618 --yaw 0.0 --frame-id base_link --child-frame-id camera_link
```

### Step 3.2: Launch the Odometry Broadcaster
Open Terminal 3 and run the dynamic circular odometry publisher from [Module 53 Chapter 07](../01-module-53-tf2-kinematics/07-ros2-practical-lab.md):
```bash
# Terminal 3: Dynamic odom -> base_link stream
ros2 run tf2_practical_lab dynamic_odom_broadcaster
```

### Step 3.3: Verify Real-Time Transformation
Open Terminal 4 and query the transformation from `odom` to `laser_frame`:
```bash
ros2 run tf2_ros tf2_echo odom laser_frame
```
Observe how the translation updates continuously at 50 Hz as the robot orbits.

### Step 3.4: Generate the TF Tree Diagram
In Terminal 4, run:
```bash
ros2 run tf2_tools view_frames
```
Open the generated `frames.pdf`:
- Verify that `odom` is the root node.
- Verify that `base_link` has parent `odom`.
- Verify that both `laser_frame` and `camera_link` have parent `base_link`.

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

## 4. Lab Challenge & Troubleshooting Exercise
Simulate an error by running:
```bash
ros2 run tf2_ros static_transform_publisher --x 1.0 --y 0.0 --z 0.0 --frame-id world --child-frame-id base_link
```
1. Observe what happens in RViz. Why does `base_link` start jittering rapidly?
2. Run `view_frames` and identify the multiple parent violation.
3. Terminate the conflicting publisher to restore tree integrity.

---

## 5. Lab Deliverables
Submit:
1. The generated `frames.pdf` showing a clean, single-parent tree.
2. A screenshot of RViz2 displaying the orbiting `base_link` and attached sensors.
3. Your written answers explaining why `world` and `odom` cannot both be parents of `base_link`.
