# Lab 02: Sensor Coordinate Transformations

## 1. Laboratory Objectives
In this laboratory, you will:
1. Reconcile computer vision optical frames ($Z$-forward, $X$-right, $Y$-down) with standard robotics frames ($X$-forward, $Y$-left, $Z$-up).
2. Build an executable Python node that transforms target detections from `camera_optical_frame` into `map` frame coordinates.
3. Test your node using simulated detections published via `ros2 topic pub`.
4. Validate coordinate calculations mathematically by hand.

---

## 2. Problem Statement
An autonomous mobile picker robot detects a soda can on a table:
- The detection node reports the can in `camera_color_optical_frame` at:
  $${}^{\text{opt}}\mathbf{p} = [0.15, \; 0.05, \; 1.20]^T\,\text{m}$$
- The camera is mounted on the robot chassis:
  - Translation: $0.25\,\text{m}$ forward ($+X$), $0.60\,\text{m}$ upward ($+Z$) relative to `base_link`.
  - Tilt: Pitched downward by $20^\circ$.
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
import tf2_geometry_msgs

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
            # Transform point directly into 'map' frame
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
        except Exception as ex:
            self.get_logger().warn(f"Transform lookup failed: {ex}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorTransformerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 4. Verification Procedure
1. Publish a test detection:
   ```bash
   ros2 topic pub --once /detected_object_camera geometry_msgs/msg/PointStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera_color_optical_frame'}, point: {x: 0.15, y: 0.05, z: 1.20}}"
   ```
2. Echo the output topic:
   ```bash
   ros2 topic echo /detected_object_map --once
   ```
3. Check the calculated numbers against your manual matrix calculation from [Module 53 Chapter 06](../01-module-53-tf2-kinematics/06-sensor-transformations.md).
