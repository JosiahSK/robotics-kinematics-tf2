# ROS 2 TF2 Command & Programming Cheat Sheet

This reference sheet provides quick lookup for ROS 2 TF2 command-line tools, command syntax across ROS 2 distributions (Jazzy Jalisco / Humble Hawksbill), Python boilerplate templates for broadcasters and listeners, and Quality of Service (QoS) configurations.

---

## 1. Essential CLI Commands

Ensure your ROS 2 environment is sourced before executing these commands:
```bash
# Ubuntu 24.04 (ROS 2 Jazzy):
source /opt/ros/jazzy/setup.bash

# Ubuntu 22.04 (ROS 2 Humble):
source /opt/ros/humble/setup.bash
```

### 1.1 Inspecting Active Frames and Transforms

| Task | Command | Notes & Description |
| :--- | :--- | :--- |
| **Echo Live Transform** | `ros2 run tf2_ros tf2_echo <parent_frame> <child_frame>` | Prints translation $[x, y, z]$, rotation in RPY $[rad, deg]$, and quaternion $[x, y, z, w]$ continuously at ~1 Hz. |
| **Generate Frame Graph PDF** | `ros2 run tf2_tools view_frames` | Listens to `/tf` and `/tf_static` for 5 seconds and generates `frames.pdf` and `frames.gv` in the current working directory. |
| **Monitor Transform Timing** | `ros2 run tf2_ros tf2_monitor <parent_frame> <child_frame>` | Measures transform publishing frequency, average delay, and maximum buffer delay. Essential for detecting frame lag. |
| **Monitor All Frames** | `ros2 run tf2_ros tf2_monitor` | Summarizes publish rates and buffer latencies for all active frame pairs in the TF tree. |
| **List Active TF Topics** | `ros2 topic list \| grep tf` | Shows `/tf` (dynamic transforms) and `/tf_static` (static transforms). |
| **Inspect Topic QoS** | `ros2 topic info -v /tf_static` | Displays durability (Transient Local vs. Volatile), reliability, and active subscriber/publisher counts. |

---

## 2. Publishing Static Transforms via CLI

Static transforms define rigid, fixed spatial relationships between robot links or mounted sensors (e.g., `base_link` $\to$ `laser_frame`).

### 2.1 Option A: Flag-Based Syntax (Recommended in ROS 2 Jazzy & Humble)

```bash
# Using Euler Angles (Yaw, Pitch, Roll in radians):
ros2 run tf2_ros static_transform_publisher \
  --x 0.25 --y 0.0 --z 0.15 \
  --yaw 0.0 --pitch 0.0 --roll 0.0 \
  --frame-id base_link --child-frame-id laser_frame

# Using Quaternions (x, y, z, w):
ros2 run tf2_ros static_transform_publisher \
  --x 0.25 --y 0.0 --z 0.15 \
  --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 \
  --frame-id base_link --child-frame-id laser_frame
```

### 2.2 Option B: Positional Argument Syntax (Legacy / Compact)

```bash
# Syntax: x y z yaw pitch roll frame_id child_frame_id
ros2 run tf2_ros static_transform_publisher 0.25 0.0 0.15 0.0 0.0 0.0 base_link laser_frame

# Syntax: x y z qx qy qz qw frame_id child_frame_id
ros2 run tf2_ros static_transform_publisher 0.25 0.0 0.15 0.0 0.0 0.0 1.0 base_link laser_frame
```

---

## 3. Python TF2 Broadcaster Boilerplates

### 3.1 Dynamic Transform Broadcaster (Timer-Driven)

Dynamic transforms change continuously over time (e.g., odometry: `odom` $\to$ `base_link`).

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np


class DynamicFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_frame_broadcaster')
        # Initialize dynamic broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        # Create 20 Hz update timer (0.05 s)
        self.timer = self.create_timer(0.05, self.broadcast_timer_callback)
        self.start_time = self.get_clock().now()

    def broadcast_timer_callback(self):
        t = TransformStamped()

        # 1. Timestamp MUST be current simulation/wall clock time
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Example trajectory: circular motion
        now_sec = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        radius = 1.0
        omega = 0.5  # rad/s

        t.transform.translation.x = radius * np.cos(omega * now_sec)
        t.transform.translation.y = radius * np.sin(omega * now_sec)
        t.transform.translation.z = 0.0

        # Orientation: heading tangent to circle (yaw = omega * t + pi/2)
        yaw = omega * now_sec + np.pi / 2.0
        # Quaternion for pure yaw: [0, 0, sin(yaw/2), cos(yaw/2)]
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = float(np.sin(yaw / 2.0))
        t.transform.rotation.w = float(np.cos(yaw / 2.0))

        # Broadcast on /tf
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 3.2 Static Transform Broadcaster (Broadcast-Once)

Static transforms are published once at node startup onto `/tf_static` with **Transient Local durability QoS**.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('static_frame_broadcaster')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

    def publish_static_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'

        # Camera mounted 20 cm forward, 35 cm up, tilted down by 15 deg
        t.transform.translation.x = 0.20
        t.transform.translation.y = 0.00
        t.transform.translation.z = 0.35

        # Pitch = +15 deg (0.2618 rad): q = [0, sin(pitch/2), 0, cos(pitch/2)]
        pitch = 0.2618
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = float(np.sin(pitch / 2.0))
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = float(np.cos(pitch / 2.0))

        # Broadcast on /tf_static (retained for late joiners)
        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info('Published static transform: base_link -> camera_link')
```

---

## 4. Python TF2 Listener Boilerplate

A TF listener maintains a temporal buffer of transforms (default: 10.0 seconds) and interpolates transforms between arbitrary frame pairs.

```python
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import (
    LookupException,
    ConnectivityException,
    ExtrapolationException
)


class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')

        # 1. Create a Buffer (cache) and attach a TransformListener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 2. Timer to query transforms periodically
        self.timer = self.create_timer(0.1, self.lookup_callback)  # 10 Hz

    def lookup_callback(self):
        target_frame = 'base_link'
        source_frame = 'camera_optical_frame'

        try:
            # Query the latest available transform (Time()):
            transform = self.tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=Time(),  # Time() means "latest available"
                timeout=Duration(seconds=0.05)  # Block up to 50ms for transform to arrive
            )

            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            self.get_logger().info(
                f'Transform [{source_frame} -> {target_frame}]: '
                f'pos=({tx:.3f}, {ty:.3f}, {tz:.3f}), quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})'
            )

        except LookupException as ex:
            self.get_logger().warn(f'Frame not found: {ex}')
        except ConnectivityException as ex:
            self.get_logger().warn(f'No connection in TF tree: {ex}')
        except ExtrapolationException as ex:
            self.get_logger().warn(f'Transform extrapolation error (timestamp out of buffer range): {ex}')
```

---

## 5. QoS Profiles: Dynamic `/tf` vs Static `/tf_static`

Understanding Quality of Service (QoS) is crucial for avoiding silent TF listener failures in ROS 2.

```
+---------------------------------------------------------------------------------------+
| Topic         | Durability       | Reliability | Depth | Notes                        |
+---------------+------------------+-------------+-------+------------------------------+
| /tf           | VOLATILE         | RELIABLE    | 100   | Discarded once delivered.   |
|               |                  |             |       | Fast continuous stream.      |
+---------------+------------------+-------------+-------+------------------------------+
| /tf_static    | TRANSIENT_LOCAL  | RELIABLE    | 1     | DDS middleware retains last  |
|               |                  |             |       | sample for late joiners.     |
+---------------------------------------------------------------------------------------+
```

> [!IMPORTANT]
> **Why `Transient Local` matters:** In ROS 1, `/tf_static` used master-side "latching". In ROS 2, there is no master node; DDS handles persistence via **Transient Local durability**. If a subscriber creates a `/tf_static` subscription with default `Volatile` durability, it will **never receive** transforms published before the subscriber started!

---

## 6. Transforming Points and Poses

To transform geometric data from a sensor frame to a robot frame, use `tf2_geometry_msgs`:

```python
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # Required to register type transforms with tf2


def transform_sensor_reading(tf_buffer, raw_point_msg, target_frame='base_link'):
    """
    Transforms a PointStamped from its header.frame_id into target_frame.
    """
    try:
        # tf_buffer.transform() automatically looks up the transform and performs
        # the SE(3) matrix-vector multiplication: p_target = T * p_source
        transformed_point = tf_buffer.transform(
            object_stamped=raw_point_msg,
            target_frame=target_frame,
            timeout=Duration(seconds=0.1)
        )
        return transformed_point
    except (LookupException, ConnectivityException, ExtrapolationException) as err:
        print(f"Transform failed: {err}")
        return None
```

---

## 7. Direct Chapter Navigation

- Conceptual introduction to coordinate frames: [01-module-53-tf2-kinematics/01-coordinate-frames.md](../../01-module-53-tf2-kinematics/01-coordinate-frames.md)
- In-depth static vs dynamic transforms & QoS: [01-module-53-tf2-kinematics/02-static-and-dynamic-transforms.md](../../01-module-53-tf2-kinematics/02-static-and-dynamic-transforms.md)
- Mobile robot standard frames (`map`, `odom`, `base_link`): [01-module-53-tf2-kinematics/03-map-odom-base-link.md](../../01-module-53-tf2-kinematics/03-map-odom-base-link.md)
- Sensor frame transformations & optical conventions: [01-module-53-tf2-kinematics/06-sensor-transformations.md](../../01-module-53-tf2-kinematics/06-sensor-transformations.md)
