# Nav2 Basic Navigation Data Transfer Stack

## Overview

This document provides a comprehensive technical reference for the core data streams and message types used in ROS2 Navigation Stack (Nav2) for basic wheeled robot navigation. This covers the essential data flow between sensors, controllers, and navigation algorithms.

## Table of Contents

1. [Core Data Flow Architecture](#core-data-flow-architecture)
2. [Essential Topics and Message Types](#essential-topics-and-message-types)
3. [Data Timing and Frequencies](#data-timing-and-frequencies)
4. [Message Structures and Fields](#message-structures-and-fields)
5. [Transform Tree (TF2)](#transform-tree-tf2)
6. [Implementation Guidelines](#implementation-guidelines)
7. [Debugging and Monitoring](#debugging-and-monitoring)

---

## Core Data Flow Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Sensors       │    │   Nav2 Stack     │    │   Actuators     │
│                 │    │                  │    │                 │
│ • Lidar (/scan) │───▶│ • Path Planning  │───▶│ • Wheels        │
│ • IMU (/imu)    │    │ • Localization   │    │   (/cmd_vel)    │
│ • Encoders      │    │ • Obstacle Avoid │    │                 │
│   (/joint_states│    │ • Control        │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                        ▲                       │
         │              ┌─────────┴─────────┐             │
         └──────────────▶│   Odometry       │◀────────────┘
                        │   (/odom)        │
                        │   (/tf)          │
                        └───────────────────┘
```

---

## Essential Topics and Message Types

### 1. Motion Control Topics

| Topic | Message Type | Direction | Purpose |
|-------|-------------|-----------|---------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 → Robot | Velocity commands to robot |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Internal | Raw navigation commands |
| `/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | Internal | Smoothed velocity commands |
| `/odom` | `nav_msgs/msg/Odometry` | Robot → Nav2 | Robot position/velocity feedback |

### 2. Sensor Topics

| Topic | Message Type | Direction | Purpose |
|-------|-------------|-----------|---------|
| `/scan` | `sensor_msgs/msg/LaserScan` | Sensor → Nav2 | Laser range data |
| `/imu` | `sensor_msgs/msg/Imu` | Sensor → Nav2 | Inertial measurement data |
| `/joint_states` | `sensor_msgs/msg/JointState` | Robot → Nav2 | Wheel encoder data |

### 3. Transform Topics

| Topic | Message Type | Direction | Purpose |
|-------|-------------|-----------|---------|
| `/tf` | `tf2_msgs/msg/TFMessage` | Bidirectional | Dynamic coordinate transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Publishers | Static coordinate transforms |

### 4. Navigation State Topics

| Topic | Message Type | Direction | Purpose |
|-------|-------------|-----------|---------|
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | AMCL → Nav2 | Localization estimate |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | User → AMCL | Initial pose estimate |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | User → Nav2 | Navigation goal |

---

## Data Timing and Frequencies

### Critical Timing Requirements

| Topic | Typical Frequency | Acceptable Range | Critical? |
|-------|------------------|------------------|-----------|
| `/cmd_vel` | 20 Hz | 10-50 Hz | ✅ YES |
| `/odom` | 50 Hz | 20-100 Hz | ✅ YES |
| `/scan` | 10 Hz | 5-40 Hz | ✅ YES |
| `/joint_states` | 50 Hz | 20-100 Hz | ✅ YES |
| `/tf` | 100+ Hz | 50-200 Hz | ✅ YES |
| `/imu` | 100 Hz | 50-500 Hz | ⚠️ OPTIONAL |

### Latency Requirements
- **Total loop latency**: < 100ms (sensor → decision → actuation)
- **Transform latency**: < 10ms
- **Control loop latency**: < 50ms

---

## Message Structures and Fields

### 1. Velocity Commands (`geometry_msgs/msg/Twist`)

```yaml
# Message structure for /cmd_vel
linear:
  x: float64    # Forward/backward velocity (m/s) [-max_vel, +max_vel]
  y: float64    # Left/right velocity (m/s) [typically 0 for diff-drive]
  z: float64    # Up/down velocity (m/s) [always 0 for ground robots]
angular:
  x: float64    # Roll rate (rad/s) [always 0 for wheeled robots]
  y: float64    # Pitch rate (rad/s) [always 0 for wheeled robots]
  z: float64    # Yaw rate (rad/s) [-max_angular_vel, +max_angular_vel]
```

**Typical Values for TurtleBot3:**
```yaml
# Example moving forward and turning right
linear:  {x: 0.26, y: 0.0, z: 0.0}   # 0.26 m/s forward
angular: {x: 0.0, y: 0.0, z: -1.82}  # 1.82 rad/s clockwise
```

### 2. Odometry Data (`nav_msgs/msg/Odometry`)

```yaml
header:
  stamp:                              # Timestamp of measurement
    sec: int32                        # Seconds since epoch
    nanosec: uint32                   # Nanoseconds component
  frame_id: string                    # "odom" - odometry reference frame

child_frame_id: string                # "base_footprint" - robot base frame

pose:                                 # Position and orientation estimate
  pose:
    position:
      x: float64                      # X position in odom frame (meters)
      y: float64                      # Y position in odom frame (meters)
      z: float64                      # Z position (typically 0.0)
    orientation:                      # Quaternion rotation
      x: float64                      # Quaternion x (typically 0.0)
      y: float64                      # Quaternion y (typically 0.0)
      z: float64                      # Quaternion z (yaw component)
      w: float64                      # Quaternion w (yaw component)
  covariance: float64[36]             # 6x6 covariance matrix (position + orientation)

twist:                                # Velocity estimate
  twist:
    linear:  {x: float64, y: float64, z: float64}   # Linear velocities
    angular: {x: float64, y: float64, z: float64}   # Angular velocities
  covariance: float64[36]             # 6x6 covariance matrix (velocity)
```

**Key Covariance Matrix Structure:**
```
[x_pos, y_pos, z_pos, x_rot, y_rot, z_rot]
[ 0-5    6-11   12-17  18-23  24-29  30-35]
```

### 3. Joint States (`sensor_msgs/msg/JointState`)

```yaml
header:
  stamp:                              # Timestamp of measurement
    sec: int32
    nanosec: uint32
  frame_id: string                    # Usually empty ""

name: string[]                        # Joint names ["wheel_left_joint", "wheel_right_joint"]
position: float64[]                   # Joint positions in radians [left_angle, right_angle]
velocity: float64[]                   # Joint velocities in rad/s [left_vel, right_vel]
effort: float64[]                     # Joint efforts in N⋅m [left_torque, right_torque]
```

**Example for Differential Drive:**
```yaml
name: ["wheel_left_joint", "wheel_right_joint"]
position: [245.67, 198.34]           # Cumulative wheel rotations (rad)
velocity: [1.25, 1.47]               # Current wheel speeds (rad/s)
effort: [0.0, 0.0]                   # Usually not used for velocity control
```

### 4. Laser Scan (`sensor_msgs/msg/LaserScan`)

```yaml
header:
  stamp: builtin_interfaces/Time      # Timestamp of scan
  frame_id: string                    # "base_scan" - laser frame

angle_min: float32                    # Start angle of scan (radians)
angle_max: float32                    # End angle of scan (radians)
angle_increment: float32              # Angular distance between measurements (radians)
time_increment: float32               # Time between measurements (seconds)
scan_time: float32                    # Time between scans (seconds)
range_min: float32                    # Minimum range value (meters)
range_max: float32                    # Maximum range value (meters)
ranges: float32[]                     # Range measurements (meters)
intensities: float32[]                # Intensity measurements (optional)
```

**Typical LiDAR Values:**
```yaml
angle_min: -3.141592653589793        # -π radians (-180°)
angle_max: 3.141592653589793         # +π radians (+180°)
angle_increment: 0.01745329252       # ~1° in radians
range_min: 0.12                      # 12 cm minimum
range_max: 3.5                       # 3.5 m maximum
ranges: [inf, 2.45, 2.67, ...]      # Array of 360 measurements
```

---

## Transform Tree (TF2)

### Standard Frame Hierarchy

```
map
 └── odom                    # Odometry frame (drifts over time)
     └── base_footprint      # Robot's ground projection
         └── base_link       # Robot's physical center
             ├── base_scan   # Laser scanner frame
             ├── imu_link    # IMU sensor frame
             ├── wheel_left_link
             └── wheel_right_link
```

### Transform Data (`tf2_msgs/msg/TFMessage`)

```yaml
transforms:
  - header:
      stamp: builtin_interfaces/Time  # Transform timestamp
      frame_id: string                # Parent frame
    child_frame_id: string            # Child frame
    transform:
      translation:
        x: float64                    # Translation in meters
        y: float64
        z: float64
      rotation:                       # Quaternion rotation
        x: float64
        y: float64
        z: float64
        w: float64
```

### Critical Transform Relationships

1. **`odom` → `base_footprint`**: Published by odometry system (wheel encoders + IMU)
2. **`map` → `odom`**: Published by localization system (AMCL)
3. **`base_footprint` → `base_link`**: Static transform (usually identity or z-offset)
4. **`base_link` → `base_scan`**: Static transform (sensor mounting position)

---

## Implementation Guidelines

### 1. Differential Drive Kinematics

**Forward Kinematics (Encoders → Odometry):**
```python
# Given wheel velocities (rad/s)
wheel_left_vel = joint_states.velocity[0]
wheel_right_vel = joint_states.velocity[1]

# Robot parameters
wheel_radius = 0.033        # meters
wheel_separation = 0.160    # meters

# Calculate robot velocities
linear_vel = wheel_radius * (wheel_right_vel + wheel_left_vel) / 2.0
angular_vel = wheel_radius * (wheel_right_vel - wheel_left_vel) / wheel_separation

# Integrate for position (dt = time step)
x += linear_vel * cos(theta) * dt
y += linear_vel * sin(theta) * dt
theta += angular_vel * dt
```

**Inverse Kinematics (Cmd_vel → Wheel Commands):**
```python
# Given desired robot motion
linear_x = cmd_vel.linear.x
angular_z = cmd_vel.angular.z

# Calculate required wheel velocities
wheel_left_vel = (linear_x - angular_z * wheel_separation / 2.0) / wheel_radius
wheel_right_vel = (linear_x + angular_z * wheel_separation / 2.0) / wheel_radius
```

### 2. Message Publishing Templates

**Odometry Publisher (Python):**
```python
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)
        
    def publish_odometry(self, x, y, theta, vx, vtheta):
        current_time = self.get_clock().now()
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        quat = tf_transformations.quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vtheta
        
        # Covariance (example values)
        odom.pose.covariance[0] = 0.1   # x position variance
        odom.pose.covariance[7] = 0.1   # y position variance
        odom.pose.covariance[35] = 0.1  # yaw variance
        
        self.odom_pub.publish(odom)
        
        # Publish transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(transform)
```

**Cmd_vel Subscriber (Python):**
```python
from geometry_msgs.msg import Twist

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to wheel velocities
        wheel_left_vel = (linear_x - angular_z * self.wheel_separation / 2.0) / self.wheel_radius
        wheel_right_vel = (linear_x + angular_z * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Send to motors
        self.send_wheel_commands(wheel_left_vel, wheel_right_vel)
```

### 3. Parameter Configuration

**Typical Nav2 Parameters for Differential Drive:**
```yaml
# Controller parameters
controller_frequency: 20.0           # Hz
min_vel_x: -0.26                     # m/s
max_vel_x: 0.26                      # m/s
min_vel_theta: -1.82                 # rad/s
max_vel_theta: 1.82                  # rad/s
acc_lim_x: 2.5                       # m/s²
acc_lim_theta: 3.2                   # rad/s²

# Robot footprint
footprint: "[[0.105, 0.105], [0.105, -0.105], [-0.105, -0.105], [-0.105, 0.105]]"

# Odometry parameters
odom_frame_id: "odom"
base_frame_id: "base_footprint"
publish_frequency: 50.0              # Hz
```

---

## Debugging and Monitoring

### 1. Topic Monitoring Commands

```bash
# List all active topics
ros2 topic list

# Monitor message frequency
ros2 topic hz /cmd_vel
ros2 topic hz /odom
ros2 topic hz /scan

# View message content
ros2 topic echo /cmd_vel --once
ros2 topic echo /odom --once

# Check topic information
ros2 topic info /cmd_vel
ros2 topic info /odom

# Visualize topic graph
rqt_graph
```

### 2. Transform Debugging

```bash
# View transform tree
ros2 run tf2_tools view_frames

# Check specific transforms
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom

# Monitor transform frequency
ros2 topic hz /tf
ros2 topic hz /tf_static
```

### 3. Performance Monitoring

```bash
# Monitor CPU usage by nodes
top -p $(pgrep -d',' -f ros2)

# Check message latency
ros2 topic echo /odom | grep -A1 "sec:"

# Bandwidth monitoring
ros2 topic bw /cmd_vel
ros2 topic bw /odom
ros2 topic bw /scan
```

### 4. Common Issues and Solutions

| Issue | Symptoms | Solution |
|-------|----------|----------|
| **Missing transforms** | Navigation not working, TF errors | Check `robot_state_publisher`, verify URDF |
| **Odometry drift** | Robot position inaccurate | Tune wheel parameters, check encoder resolution |
| **Cmd_vel not received** | Robot not moving | Check topic remapping, verify subscriber |
| **High latency** | Jerky movement | Increase publish frequencies, optimize code |
| **Covariance too high** | Localization unstable | Tune odometry covariance parameters |

---

## Reference Implementation Checklist

### For Robot Hardware Interface:
- [ ] Subscribe to `/cmd_vel` at ≥20 Hz
- [ ] Publish `/odom` at ≥50 Hz
- [ ] Publish `/joint_states` at ≥50 Hz
- [ ] Publish `odom` → `base_footprint` transform
- [ ] Handle velocity limits and acceleration limits
- [ ] Implement proper error recovery

### For Sensor Integration:
- [ ] Publish `/scan` at ≥10 Hz
- [ ] Publish `/imu` if available
- [ ] Publish static transforms for sensors
- [ ] Synchronize timestamps across sensors
- [ ] Handle sensor failure gracefully

### For Navigation Integration:
- [ ] Verify transform tree completeness
- [ ] Validate coordinate frame conventions
- [ ] Test emergency stop functionality
- [ ] Implement diagnostics publishing
- [ ] Verify parameter configuration

---

## Additional Resources

- **ROS2 Nav2 Documentation**: https://docs.nav2.org/
- **TF2 Tutorials**: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/
- **Message Definitions**: https://docs.ros.org/en/jazzy/p/common_interfaces/
- **Differential Drive Tutorial**: https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html

---

*Last Updated: July 28, 2025*
*Compatible with: ROS2 Jazzy, Nav2 1.3+*
