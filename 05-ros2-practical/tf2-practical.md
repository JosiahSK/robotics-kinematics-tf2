# TF2 in Practice: Software Architecture & Implementation

## 1. Learning Objectives
By completing this chapter, you will be able to:
- Structure real-time ROS 2 nodes using `tf2_ros.Buffer` and `TransformListener`.
- Synchronize incoming sensor topics with coordinate transforms using `tf2_ros.MessageFilter`.
- Avoid common latency and threading bottlenecks when querying transforms.
- Handle TF timeouts and exceptions safely without crashing the control loop.

---

## 2. The TF2 Software Architecture

TF2 is not a single node; it is a **distributed client-server architecture** built on top of the ROS 2 DDS middleware:

```mermaid
graph TD
    subgraph Broadcasters ["Transform Broadcasters"]
        B1["robot_state_publisher"] -->|/tf_static (Latched)| DDS["ROS 2 DDS Middleware"]
        B2["wheel_odometry_node"] -->|/tf (50 Hz)| DDS
        B3["slam_toolbox"] -->|/tf (5 Hz)| DDS
    end
    subgraph Subscribers ["Transform Listeners"]
        DDS --> L1["TransformListener (Node A)"]
        DDS --> L2["TransformListener (Node B)"]
        L1 --> Buf1["In-Memory Ring Buffer (10.0 sec)"]
        L2 --> Buf2["In-Memory Ring Buffer (10.0 sec)"]
    end
```

Every node that instantiates a `TransformListener`:
1. Automatically creates internal ROS 2 subscriptions to `/tf` and `/tf_static`.
2. Stores incoming transforms in a local, thread-safe in-memory ring buffer (default cache time: $10.0\,\text{seconds}$).
3. When you call `lookup_transform()`, **no network request is sent**! The query is solved locally in memory using fast tree search and SLERP interpolation.

---

## 3. The `MessageFilter` Pattern: Perfect Sensor Synchronization

A common problem in robotics perception:
- A camera publishes a PointCloud at timestamp $t = 12.350\,\text{s}$.
- The subscriber receives the PointCloud at $t = 12.355\,\text{s}$.
- If the subscriber calls `lookup_transform('map', 'camera', time=12.350)` immediately, the transform might not have arrived yet over the network!

If you write a naive sleep loop, you block the executor.
The professional ROS 2 solution is the **`tf2_ros.MessageFilter`**:
- The filter holds the sensor message in a queue.
- It constantly checks the TF buffer.
- The instant the transform spanning timestamp $t = 12.350\,\text{s}$ arrives, the filter triggers your callback!

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import tf2_ros
from tf2_ros import MessageFilter
from message_filters import Subscriber

class SyncedPointCloudNode(Node):
    def __init__(self):
        super().__init__('synced_pointcloud_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 1. Subscribe to the sensor topic
        self.sub = Subscriber(self, PointCloud2, '/camera/points')

        # 2. Wrap in TF2 MessageFilter
        self.filter = MessageFilter(
            self.sub,
            self.tf_buffer,
            target_frame='map',
            queue_size=10,
            node=self
        )

        # 3. Register callback: only called when transform to 'map' is READY!
        self.filter.registerCallback(self.pointcloud_callback)

    def pointcloud_callback(self, msg: PointCloud2):
        self.get_logger().info(f"Received PointCloud perfectly synchronized to 'map' at stamp {msg.header.stamp.sec}")
```

---

## 4. Summary & Best Practices
- Never query transforms with zero timeout inside callbacks.
- Use `MessageFilter` for high-bandwidth sensor streams.
- Next, explore how the robot's physical links are tracked in [**The Robot State Publisher**](robot-state-publisher.md).
