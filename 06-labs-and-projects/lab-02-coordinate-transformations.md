# Lab 02: Sensor Coordinate Transformations

> **Prerequisites:** [06: Sensor Data Transformations](../01-module-53-tf2-kinematics/06-sensor-transformations.md), [04: Homogeneous Transformations](../02-module-54-mathematics/04-homogeneous-transformations.md), ROS 2 environment.  
> **Estimated Time:** 45–60 minutes  
> **Code Files:**
> - Starter Template: Python node boilerplate in Section 3
> - Reference Concept: [Sensor Transformations Chapter](../01-module-53-tf2-kinematics/06-sensor-transformations.md)

---

## 1. Laboratory Objectives
In this laboratory, you will:
1. Reconcile computer vision optical frames ($Z$-forward, $X$-right, $Y$-down) with standard robotics frames ($X$-forward, $Y$-left, $Z$-up).
2. Build an executable Python node that transforms target detections from `camera_optical_frame` into global `map` frame coordinates.
3. Test your node using simulated detections published via `ros2 topic pub`.
4. Validate coordinate calculations mathematically against hand calculations.

---

## 2. Problem Statement
An autonomous mobile picker robot detects an object on a warehouse shelf:
- The detection node reports the object in `camera_color_optical_frame` at:
  $${}^{\text{opt}}\mathbf{p} = [0.15, \; 0.05, \; 1.20]^T\,\text{m}$$
- The camera is mounted on the robot chassis:
  - Translation: $0.25\,\text{m}$ forward ($+X$), $0.60\,\text{m}$ upward ($+Z$) relative to `base_link`.
  - Tilt: Pitched downward by $20^\circ$ ($0.3491\,\text{rad}$).
- The robot is located in the global `map` at:
  - Position: $(x = 3.50\,\text{m}, y = 2.00\,\text{m})$.
  - Heading: $\theta = 90^\circ$ ($\pi/2\,\text{rad}$ yaw).

**Your Task:** Write a ROS 2 node that subscribes to `geometry_msgs/msg/PointStamped` on `/detected_object_camera` and publishes the transformed point in `/detected_object_map`.

---

## 3. Node Implementation: `sensor_transformer_node.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  # Required to register transform methods


class SensorTransformerNode(Node):
    def __init__(self):
        super().__init__('sensor_transformer_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointStamped, '/detected_object_camera', self.detection_callback, 10
        )
        self.pub = self.create_publisher(
            PointStamped, '/detected_object_map', 10
        )
        self.get_logger().info("Sensor Transformer Node started. Ready for detections.")

    def detection_callback(self, msg: PointStamped):
        try:
            # Transform point directly into 'map' frame with 200ms lookup timeout
            map_point = self.tf_buffer.transform(
                msg,
                target_frame='map',
                timeout=Duration(seconds=0.2)
            )
            self.pub.publish(map_point)
            self.get_logger().info(
                f"[Transformed to Map] X={map_point.point.x:.3f}m, "
                f"Y={map_point.point.y:.3f}m, Z={map_point.point.z:.3f}m"
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(f"Transform lookup failed: {ex}")


def main(args=None):
    rclpy.init(args=args)
    node = SensorTransformerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 4. Step-by-Step Task Sequence & Run Commands

### Step 4.1: Publish the Coordinate Tree
Open Terminal 1 and publish static transforms for `map` $\to$ `base_link` and `base_link` $\to$ `camera_color_optical_frame`:
```bash
# Map to base_link (x=3.5, y=2.0, yaw=90 deg)
ros2 run tf2_ros static_transform_publisher \
  --x 3.50 --y 2.00 --z 0.0 \
  --yaw 1.5708 --pitch 0.0 --roll 0.0 \
  --frame-id map --child-frame-id base_link

# base_link to camera_color_optical_frame
# (Pitch 20 deg down plus standard optical frame rotation)
ros2 run tf2_ros static_transform_publisher \
  --x 0.25 --y 0.0 --z 0.60 \
  --yaw -1.5708 --pitch 0.0 --roll -2.0071 \
  --frame-id base_link --child-frame-id camera_color_optical_frame
```

### Step 4.2: Start the Sensor Transformer Node
In Terminal 2:
```bash
python3 sensor_transformer_node.py
```

### Step 4.3: Publish Test Detection
In Terminal 3, publish a simulated soda can detection:
```bash
ros2 topic pub --once /detected_object_camera geometry_msgs/msg/PointStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera_color_optical_frame'}, point: {x: 0.15, y: 0.05, z: 1.20}}"
```

### Step 4.4: Echo the Transformed Coordinates
```bash
ros2 topic echo /detected_object_map --once
```

#### Expected Output:
```text
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: map
point:
  x: 3.650
  y: 3.390
  z: 0.190
```

---

## 5. Verification Checkpoints
- [ ] Node gracefully handles empty / initial timestamps by querying `tf_buffer` with timeout.
- [ ] Transformed coordinates match the physical intuition: since the robot faces $+Y$ (yaw $90^\circ$), objects in front of the camera must increase the $Y_{\text{map}}$ coordinate.
- [ ] If a transform is temporarily unavailable, node logs a warning instead of crashing.

---

## 6. Extension Task
Add **sensor uncertainty propagation** (from [Module 54 Chapter 09](../02-module-54-mathematics/09-covariance-matrices.md)):
- A camera depth measurement has an uncertainty covariance $\boldsymbol{\Sigma}_{\text{opt}} = \operatorname{diag}(0.01^2, 0.01^2, 0.05^2)\,\text{m}^2$.
- Transform the covariance matrix into the global map frame using the rotation matrix $\mathbf{R}_{\text{opt}}^{\text{map}}$:
  $$\boldsymbol{\Sigma}_{\text{map}} = \mathbf{R}_{\text{opt}}^{\text{map}} \; \boldsymbol{\Sigma}_{\text{opt}} \; (\mathbf{R}_{\text{opt}}^{\text{map}})^T$$
- Print the eigenvalues of $\boldsymbol{\Sigma}_{\text{map}}$ and verify that variance is preserved under rotation ($\operatorname{trace}(\boldsymbol{\Sigma}_{\text{map}}) = \operatorname{trace}(\boldsymbol{\Sigma}_{\text{opt}})$).

---

## 7. Lab Deliverables
Submit:
1. Your verified `sensor_transformer_node.py` script.
2. Console log showing successful reception and transformation of the test point.
3. Your step-by-step mathematical derivation showing how the camera pitch and robot heading produce the final $(x, y, z)_{\text{map}}$ position.
