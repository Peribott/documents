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
# Example 1: Moving straight forward
linear:  {x: 0.22, y: 0.0, z: 0.0}   # 0.22 m/s forward
angular: {x: 0.0, y: 0.0, z: 0.0}    # No rotation

# Example 2: Turning in place (rotate left)
linear:  {x: 0.0, y: 0.0, z: 0.0}    # No forward motion
angular: {x: 0.0, y: 0.0, z: 1.82}   # 1.82 rad/s counter-clockwise

# Example 3: Forward and turning right
linear:  {x: 0.26, y: 0.0, z: 0.0}   # 0.26 m/s forward
angular: {x: 0.0, y: 0.0, z: -1.82}  # 1.82 rad/s clockwise

# Example 4: Backing up while turning left
linear:  {x: -0.15, y: 0.0, z: 0.0}  # 0.15 m/s backward
angular: {x: 0.0, y: 0.0, z: 0.8}    # 0.8 rad/s counter-clockwise

# Example 5: Emergency stop
linear:  {x: 0.0, y: 0.0, z: 0.0}    # All velocities zero
angular: {x: 0.0, y: 0.0, z: 0.0}
```

**Real-time Example from Running Robot:**
```bash
# Command to see actual cmd_vel messages:
$ ros2 topic echo /cmd_vel --once
linear:
  x: 0.05263926461338997
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.028556108474731445
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

**Real Odometry Example from TurtleBot3:**
```yaml
# Example 1: Robot at origin, stationary
header: {stamp: {sec: 100, nanosec: 250000000}, frame_id: "odom"}
child_frame_id: "base_footprint"
pose:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}  # Facing forward
  covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,      # x variance = 0.1
               0.0, 0.1, 0.0, 0.0, 0.0, 0.0,      # y variance = 0.1
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,      # z (unused)
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,      # roll (unused)
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,      # pitch (unused)
               0.0, 0.0, 0.0, 0.0, 0.0, 0.1]      # yaw variance = 0.1
twist:
  twist:
    linear: {x: 0.0, y: 0.0, z: 0.0}
    angular: {x: 0.0, y: 0.0, z: 0.0}

# Example 2: Robot moving forward at 0.2 m/s, rotated 45°
header: {stamp: {sec: 150, nanosec: 500000000}, frame_id: "odom"}
child_frame_id: "base_footprint"
pose:
  pose:
    position: {x: 2.5, y: 1.8, z: 0.0}           # 2.5m forward, 1.8m left
    orientation: {x: 0.0, y: 0.0, z: 0.383, w: 0.924}  # 45° rotation
  covariance: [0.15, 0.05, 0.0, 0.0, 0.0, 0.0,   # Higher uncertainty after movement
               0.05, 0.15, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.2]
twist:
  twist:
    linear: {x: 0.2, y: 0.0, z: 0.0}           # Moving forward at 0.2 m/s
    angular: {x: 0.0, y: 0.0, z: 0.0}          # Not rotating
```

**Live Example from Running Robot:**
```bash
$ ros2 topic echo /odom --once
header:
  stamp: {sec: 5735, nanosec: 508000000}
  frame_id: odom
child_frame_id: base_footprint
pose:
  pose:
    position: {x: 3.2133248354429087, y: 1.065818647813844, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.9909378408223198, w: 0.134321240414161}
  covariance:
  - 0.0  # x-x covariance
  - 0.0  # x-y covariance  
  - 0.0  # x-z covariance
  - 0.0  # x-roll covariance
  - 0.0  # x-pitch covariance
  - 0.0  # x-yaw covariance
  - 0.0  # y-x covariance
  - 0.0  # y-y covariance
  - 0.0  # y-z covariance
  - 0.0  # y-roll covariance
  - 0.0  # y-pitch covariance
  - 0.0  # y-yaw covariance
  - 0.0  # z-x covariance
  - 0.0  # z-y covariance
  - 0.0  # z-z covariance
  - 0.0  # z-roll covariance
  - 0.0  # z-pitch covariance
  - 0.0  # z-yaw covariance
  - 0.0  # roll-x covariance
  - 0.0  # roll-y covariance
  - 0.0  # roll-z covariance
  - 0.0  # roll-roll covariance
  - 0.0  # roll-pitch covariance
  - 0.0  # roll-yaw covariance
  - 0.0  # pitch-x covariance
  - 0.0  # pitch-y covariance
  - 0.0  # pitch-z covariance
  - 0.0  # pitch-roll covariance
  - 0.0  # pitch-pitch covariance
  - 0.0  # pitch-yaw covariance
  - 0.0  # yaw-x covariance
  - 0.0  # yaw-y covariance
  - 0.0  # yaw-z covariance
  - 0.0  # yaw-roll covariance
  - 0.0  # yaw-pitch covariance
  - 0.0  # yaw-yaw covariance
twist:
  twist:
    linear: {x: 0.0, y: 0.0, z: 0.0}         # Robot currently stopped
    angular: {x: 0.0, y: 0.0, z: 0.0}        # Not rotating
  covariance:
  - 0.0  # vx-vx covariance
  - 0.0  # vx-vy covariance
  - 0.0  # vx-vz covariance
  - 0.0  # vx-vroll covariance
  - 0.0  # vx-vpitch covariance
  - 0.0  # vx-vyaw covariance
  - 0.0  # vy-vx covariance
  - 0.0  # vy-vy covariance
  - 0.0  # vy-vz covariance
  - 0.0  # vy-vroll covariance
  - 0.0  # vy-vpitch covariance
  - 0.0  # vy-vyaw covariance
  - 0.0  # vz-vx covariance
  - 0.0  # vz-vy covariance
  - 0.0  # vz-vz covariance
  - 0.0  # vz-vroll covariance
  - 0.0  # vz-vpitch covariance
  - 0.0  # vz-vyaw covariance
  - 0.0  # vroll-vx covariance
  - 0.0  # vroll-vy covariance
  - 0.0  # vroll-vz covariance
  - 0.0  # vroll-vroll covariance
  - 0.0  # vroll-vpitch covariance
  - 0.0  # vroll-vyaw covariance
  - 0.0  # vpitch-vx covariance
  - 0.0  # vpitch-vy covariance
  - 0.0  # vpitch-vz covariance
  - 0.0  # vpitch-vroll covariance
  - 0.0  # vpitch-vpitch covariance
  - 0.0  # vpitch-vyaw covariance
  - 0.0  # vyaw-vx covariance
  - 0.0  # vyaw-vy covariance
  - 0.0  # vyaw-vz covariance
  - 0.0  # vyaw-vroll covariance
  - 0.0  # vyaw-vpitch covariance
  - 0.0  # vyaw-vyaw covariance
---

# Analysis of live data:
# Position: Robot is at (3.21m, 1.07m) in the odom frame
# Orientation: z=0.991, w=0.134 → yaw ≈ 2.69 radians (154°)
# Velocity: Currently stationary (all zeros)
# Covariance: All zeros indicates perfect confidence (simulation data)
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
# Example 1: Robot stationary
name: ["wheel_left_joint", "wheel_right_joint"]
position: [0.0, 0.0]                  # Wheels at initial position
velocity: [0.0, 0.0]                  # Not moving
effort: [0.0, 0.0]                    # No torque applied

# Example 2: Robot moving forward (both wheels same speed)
name: ["wheel_left_joint", "wheel_right_joint"]
position: [12.56, 12.56]              # Both wheels rotated 2π radians (1 full rotation)
velocity: [2.0, 2.0]                  # Both wheels at 2 rad/s
effort: [0.5, 0.5]                    # Equal torque to both wheels

# Example 3: Robot turning left (right wheel faster)
name: ["wheel_left_joint", "wheel_right_joint"]
position: [25.67, 31.42]              # Right wheel has rotated more
velocity: [1.0, 3.0]                  # Right wheel 3x faster than left
effort: [0.3, 0.8]                    # More torque on right wheel

# Example 4: Robot backing up and turning right
name: ["wheel_left_joint", "wheel_right_joint"]
position: [45.2, 42.1]                # Left wheel moved more (backing + turning)
velocity: [-2.5, -1.5]                # Both negative (backing), left faster
effort: [-0.6, -0.4]                  # Negative torque for reverse
```

**Live Example from Running TurtleBot3:**
```bash
$ ros2 topic echo /joint_states --once
header:
  stamp: {sec: 5756, nanosec: 583000000}
  frame_id: ''
name: [wheel_left_joint, wheel_right_joint]
position: [1786.76, 2030.32]          # Cumulative rotations in radians
velocity: [4.17e-18, 1.05e-16]        # Nearly zero (stationary)
effort: [0.0, 0.0]
```

**Converting Joint States to Robot Motion:**
```python
# Example calculation from joint states
wheel_radius = 0.033  # meters
wheel_separation = 0.160  # meters

# From the live example above:
left_vel = 4.17e-18   # rad/s (essentially 0)
right_vel = 1.05e-16  # rad/s (essentially 0)

# Calculate robot linear and angular velocity
linear_vel = wheel_radius * (right_vel + left_vel) / 2.0
# = 0.033 * (1.05e-16 + 4.17e-18) / 2.0 ≈ 0.0 m/s

angular_vel = wheel_radius * (right_vel - left_vel) / wheel_separation
# = 0.033 * (1.05e-16 - 4.17e-18) / 0.160 ≈ 0.0 rad/s

# Result: Robot is stationary
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
# Example 1: RPLIDAR A1M8 (360° scan)
angle_min: -3.141592653589793        # -π radians (-180°)
angle_max: 3.141592653589793         # +π radians (+180°)
angle_increment: 0.01745329252       # ~1° in radians (π/180)
time_increment: 2.777e-05            # Time between rays (~36μs)
scan_time: 0.1                       # 10 Hz scan rate
range_min: 0.12                      # 12 cm minimum
range_max: 3.5                       # 3.5 m maximum
ranges: [inf, 2.45, 2.67, 1.23, inf, 0.85, ...]  # 360 measurements
intensities: []                      # Empty if not supported

# Example 2: Hokuyo URG-04LX-UG01 (270° scan)
angle_min: -2.094395102393195        # -120° in radians
angle_max: 2.094395102393195         # +120° in radians  
angle_increment: 0.00613592315       # ~0.35° in radians
range_min: 0.06                      # 6 cm minimum
range_max: 4.0                       # 4 m maximum
ranges: [0.85, 0.86, 0.87, inf, 2.1, ...]  # 683 measurements
```

**Real Scan Data Examples:**
```yaml
# Scenario 1: Robot in open space
ranges: [3.2, 3.5, 3.4, 3.1, 2.9, inf, inf, ...]  # Mostly far readings

# Scenario 2: Robot near wall on the right
ranges: [inf, inf, 3.2, 2.8, 2.1, 1.5, 0.8, 0.5, ...]  # Close readings on right side

# Scenario 3: Robot in narrow corridor
ranges: [0.6, 0.65, 0.7, 0.8, 1.2, 1.8, 2.1, 1.9, 1.5, 0.9, 0.7, 0.6, ...]  # Close on both sides

# Scenario 4: Robot facing corner
ranges: [1.2, 1.0, 0.8, 0.6, 0.5, 0.45, 0.5, 0.6, 0.8, 1.0, 1.2, ...]  # V-shaped pattern
```

**Processing Scan Data Example:**
```python
def analyze_scan(scan_msg):
    """Example function to analyze laser scan data"""
    ranges = scan_msg.ranges
    angle_min = scan_msg.angle_min
    angle_increment = scan_msg.angle_increment
    
    # Find closest obstacle
    min_range = min([r for r in ranges if r != float('inf')])
    min_index = ranges.index(min_range)
    closest_angle = angle_min + min_index * angle_increment
    
    print(f"Closest obstacle: {min_range:.2f}m at {math.degrees(closest_angle):.1f}°")
    
    # Check if path ahead is clear (±30° from forward)
    forward_start = len(ranges) // 2 - 15  # ~30° left of center
    forward_end = len(ranges) // 2 + 15    # ~30° right of center
    forward_ranges = ranges[forward_start:forward_end]
    
    min_forward = min([r for r in forward_ranges if r != float('inf')])
    if min_forward > 1.0:
        print("Path ahead is clear")
    else:
        print(f"Obstacle ahead at {min_forward:.2f}m")
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

**Transform Examples:**

```yaml
# Example 1: odom → base_footprint (dynamic, from wheel odometry)
header: {stamp: {sec: 1000, nanosec: 500000000}, frame_id: "odom"}
child_frame_id: "base_footprint"
transform:
  translation: {x: 2.37, y: -0.11, z: 0.0}    # Robot moved 2.37m forward, 0.11m right
  rotation: {x: 0.0, y: 0.0, z: 0.89, w: 0.45} # ~116° rotation

# Example 2: map → odom (from AMCL localization correction)
header: {stamp: {sec: 1000, nanosec: 500000000}, frame_id: "map"}
child_frame_id: "odom"
transform:
  translation: {x: -0.05, y: 0.12, z: 0.0}    # Small correction from localization
  rotation: {x: 0.0, y: 0.0, z: -0.02, w: 0.999} # ~2.3° heading correction

# Example 3: base_link → base_scan (static, sensor mounting)
header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}
child_frame_id: "base_scan"
transform:
  translation: {x: 0.032, y: 0.0, z: 0.172}   # LiDAR 3.2cm forward, 17.2cm up
  rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}  # No rotation

# Example 4: base_footprint → base_link (static, ground to center)
header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_footprint"}
child_frame_id: "base_link"
transform:
  translation: {x: 0.0, y: 0.0, z: 0.01}      # Robot center 1cm above ground
  rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}  # No rotation
```

**Complete Transform Chain Example:**
```bash
# To find laser point in map frame, the chain is:
# map → odom → base_footprint → base_link → base_scan → laser_point

# Example calculation for a laser point at (1.5, 0.0) in base_scan frame:
# 1. base_scan → base_link: (1.5 + 0.032, 0.0, 0.172) = (1.532, 0.0, 0.172)
# 2. base_link → base_footprint: (1.532, 0.0, 0.172 + 0.01) = (1.532, 0.0, 0.182)
# 3. base_footprint → odom: Apply robot's current pose transform
# 4. odom → map: Apply localization correction
# Result: Point in global map coordinates
```

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

# Example with real values:
# wheel_left_vel = 2.0 rad/s, wheel_right_vel = 3.0 rad/s, dt = 0.02s
# linear_vel = 0.033 * (3.0 + 2.0) / 2.0 = 0.0825 m/s
# angular_vel = 0.033 * (3.0 - 2.0) / 0.160 = 0.206 rad/s
# If theta = 0 (facing forward):
# x += 0.0825 * cos(0) * 0.02 = 0.00165 m
# y += 0.0825 * sin(0) * 0.02 = 0.0 m  
# theta += 0.206 * 0.02 = 0.00412 rad (0.24°)
```

**Inverse Kinematics (Cmd_vel → Wheel Commands):**
```python
# Given desired robot motion
linear_x = cmd_vel.linear.x
angular_z = cmd_vel.angular.z

# Calculate required wheel velocities
wheel_left_vel = (linear_x - angular_z * wheel_separation / 2.0) / wheel_radius
wheel_right_vel = (linear_x + angular_z * wheel_separation / 2.0) / wheel_radius

# Example calculations:
# Case 1: Move forward at 0.2 m/s, no rotation
# linear_x = 0.2, angular_z = 0.0
# wheel_left_vel = (0.2 - 0.0 * 0.160/2.0) / 0.033 = 6.06 rad/s
# wheel_right_vel = (0.2 + 0.0 * 0.160/2.0) / 0.033 = 6.06 rad/s

# Case 2: Turn in place at 1.0 rad/s
# linear_x = 0.0, angular_z = 1.0
# wheel_left_vel = (0.0 - 1.0 * 0.160/2.0) / 0.033 = -2.42 rad/s
# wheel_right_vel = (0.0 + 1.0 * 0.160/2.0) / 0.033 = 2.42 rad/s

# Case 3: Move forward 0.15 m/s while turning left 0.5 rad/s
# linear_x = 0.15, angular_z = 0.5
# wheel_left_vel = (0.15 - 0.5 * 0.080) / 0.033 = 3.33 rad/s
# wheel_right_vel = (0.15 + 0.5 * 0.080) / 0.033 = 5.76 rad/s
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

**Example Parameter Files:**

**TurtleBot3 Burger Parameters:**
```yaml
# turtlebot3_burger.yaml
robot_model_type: "differential"
wheel_radius: 0.033
wheel_separation: 0.160
max_linear_velocity: 0.22
max_angular_velocity: 2.84
min_turning_radius: 0.0

# Physical constraints
max_linear_acceleration: 2.5
max_angular_acceleration: 3.2
max_linear_deceleration: -2.5
max_angular_deceleration: -3.2

# Controller tuning
xy_goal_tolerance: 0.05              # 5cm position tolerance
yaw_goal_tolerance: 0.05             # ~3° orientation tolerance
transform_tolerance: 0.2             # 200ms transform timeout
```

**Custom Robot Parameters:**
```yaml
# custom_robot.yaml
robot_model_type: "differential"
wheel_radius: 0.050                  # Larger wheels
wheel_separation: 0.250              # Wider wheelbase
max_linear_velocity: 0.5             # Faster robot
max_angular_velocity: 1.5            # Slower turning

# Safety margins
inflation_radius: 0.25               # 25cm safety bubble
cost_scaling_factor: 3.0             # Higher penalty for obstacles

# Performance tuning
controller_frequency: 10.0           # Lower frequency for slower MCU
publish_frequency: 30.0              # Moderate odometry rate
planner_frequency: 1.0               # Re-plan every second
```

**Parameter Validation Example:**
```bash
# Check if parameters are loaded correctly
ros2 param list /controller_server
ros2 param get /controller_server max_vel_x
ros2 param get /controller_server wheel_separation

# Example output:
# Double value is: 0.26
# Double value is: 0.160
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

**Example Debugging Session:**
```bash
# Step 1: Check if robot is receiving commands
$ ros2 topic hz /cmd_vel
average rate: 20.020
	min: 0.027s max: 0.084s std dev: 0.01938s window: 22

# Step 2: See what commands are being sent
$ ros2 topic echo /cmd_vel --once
linear: {x: 0.053, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.029}

# Step 3: Check if odometry is being published
$ ros2 topic hz /odom
average rate: 50.015
	min: 0.019s max: 0.021s std dev: 0.00051s window: 52

# Step 4: Verify robot is actually moving according to odometry
$ ros2 topic echo /odom --once | grep -A3 "twist:"
twist:
  twist:
    linear: {x: 0.052, y: 0.0, z: 0.0}      # Good: matches cmd_vel
    angular: {x: 0.0, y: 0.0, z: 0.028}     # Good: matches cmd_vel

# Step 5: Check bandwidth usage
$ ros2 topic bw /cmd_vel
subscribed to [/cmd_vel]
average: 1.37KB/s
	mean: 0.07KB, min: 0.07KB max: 0.07KB window: 20

# Step 6: Find nodes publishing/subscribing to topics
$ ros2 topic info /cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 2
Subscription count: 1

Publishers:
 * /bt_navigator (rmw_fastrtps_cpp)
 * /velocity_smoother (rmw_fastrtps_cpp)

Subscriptions:
 * /turtlebot3_node (rmw_fastrtps_cpp)
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

**Example Transform Debugging:**
```bash
# Check if transforms are being published
$ ros2 topic hz /tf
average rate: 127.932
	min: 0.007s max: 0.009s std dev: 0.00041s window: 130

# Verify specific transform chain
$ ros2 run tf2_ros tf2_echo odom base_footprint
At time 1690547123.456
- Translation: [2.345, -0.123, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.707, 0.707]
- Rotation: in RPY (radian) [0.000, -0.000, 1.571]
- Rotation: in RPY (degree) [0.000, -0.000, 90.000]

# Check for missing transforms
$ ros2 run tf2_ros tf2_echo map base_footprint
[ERROR] [1690547150.123]: Lookup would require extrapolation into the past.
[ERROR] [1690547150.456]: Could not transform from map to base_footprint

# Generate transform tree visualization
$ ros2 run tf2_tools view_frames
# Creates frames.pdf showing the complete transform tree

# Example tf tree output:
# map
#  └─ odom
#      └─ base_footprint
#          └─ base_link
#              ├─ wheel_left_link
#              ├─ wheel_right_link
#              ├─ base_scan
#              └─ imu_link

# Check static transforms
$ ros2 topic echo /tf_static --once
transforms:
- header:
    stamp: {sec: 0, nanosec: 0}
    frame_id: base_link
  child_frame_id: base_scan
  transform:
    translation: {x: 0.032, y: 0.0, z: 0.172}
    rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
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

**Example Performance Analysis:**
```bash
# CPU usage monitoring
$ top -p $(pgrep -d',' -f ros2)
  PID USER      PR  NI    VIRT    RES    SHR S  %CPU %MEM     TIME+ COMMAND
13310 dhruvel   20   0 3842552 275032 170644 S  45.0  3.4  13:36.89 component_container
13307 dhruvel   20   0  822928  37444  29876 S  13.6  0.4   1:16.23 robot_state_pub
13309 dhruvel   20   0 3139044 225528 150632 S  12.5  2.8   4:26.45 rviz2
13303 dhruvel   20   0 1163996  58932  47844 S   8.9  0.7   2:36.12 parameter_bridge

# Message latency check
$ ros2 topic echo /odom | grep -A1 "sec:" | head -10
    sec: 5894
    nanosec: 167000000
--
    sec: 5894
    nanosec: 187000000
# Latency = 20ms between messages (50 Hz rate confirmed)

# Bandwidth analysis
$ ros2 topic bw /scan
subscribed to [/scan]
average: 115.23KB/s
	mean: 11.52KB, min: 11.45KB max: 11.67KB window: 10

$ ros2 topic bw /odom  
subscribed to [/odom]
average: 24.67KB/s
	mean: 0.49KB, min: 0.49KB max: 0.49KB window: 50

$ ros2 topic bw /cmd_vel
subscribed to [/cmd_vel]
average: 1.37KB/s
	mean: 0.07KB, min: 0.07KB max: 0.07KB window: 20

# Network usage summary:
# /scan:     115 KB/s (highest - point cloud data)
# /odom:      25 KB/s (moderate - pose + covariance)
# /cmd_vel:    1 KB/s (lowest - simple velocity commands)
# Total:    ~141 KB/s for basic navigation

# Memory usage by ROS nodes
$ ps aux --sort=-%mem | grep ros2 | head -5
dhruvel  13310 45.0  3.4 3842552 275032 pts/1 Sl+ 13:19 nav2_container
dhruvel  13309 12.5  2.8 3139044 225528 pts/1 Sl+ 13:19 rviz2  
dhruvel  13303  8.9  0.7 1163996  58932 pts/1 Sl+ 13:19 gz_bridge
dhruvel  13307 13.6  0.4  822928  37444 pts/1 Sl+ 13:19 robot_state_pub
```

### 4. Common Issues and Solutions

| Issue | Symptoms | Solution |
|-------|----------|----------|
| **Missing transforms** | Navigation not working, TF errors | Check `robot_state_publisher`, verify URDF |
| **Odometry drift** | Robot position inaccurate | Tune wheel parameters, check encoder resolution |
| **Cmd_vel not received** | Robot not moving | Check topic remapping, verify subscriber |
| **High latency** | Jerky movement | Increase publish frequencies, optimize code |
| **Covariance too high** | Localization unstable | Tune odometry covariance parameters |

**Detailed Troubleshooting Examples:**

**Problem 1: Robot not moving despite cmd_vel commands**
```bash
# Check if commands are being published
$ ros2 topic echo /cmd_vel --once
linear: {x: 0.2, y: 0.0, z: 0.0}  # Commands are present

# Check if robot hardware is subscribed
$ ros2 topic info /cmd_vel
Subscriptions:
 * /robot_driver (rmw_fastrtps_cpp)  # Good: hardware is listening

# Check if robot is actually trying to move
$ ros2 topic echo /joint_states --once | grep velocity
velocity: [0.0, 0.0]  # Problem: wheels not turning

# Solution: Check motor driver, power supply, or communication
```

**Problem 2: Odometry drifting badly**
```bash
# Check wheel parameters in config
$ ros2 param get /robot_node wheel_radius
Double value is: 0.035  # Should be 0.033 for TurtleBot3

# Check encoder resolution
$ ros2 topic echo /joint_states
position: [100.5, 99.8]  # After 1 full rotation, should be ~6.28
# Problem: Encoder scaling incorrect

# Solution: Calibrate wheel_radius and encoder_ticks_per_revolution

# Correct parameters for TurtleBot3:
wheel_radius: 0.033
wheel_separation: 0.160  
encoder_resolution: 4096  # ticks per revolution
```

**Problem 3: High message latency causing jerky motion**
```bash
# Measure current latencies
$ ros2 topic hz /cmd_vel
average rate: 5.23  # Too low! Should be ~20 Hz

$ ros2 topic hz /odom  
average rate: 12.45  # Too low! Should be ~50 Hz

# Check system load
$ top
%Cpu(s): 95.2 us,  2.1 sy  # CPU overloaded

# Solutions:
# 1. Reduce rviz update rate
# 2. Lower scan frequency if possible  
# 3. Optimize code in control loops
# 4. Use faster hardware
```

**Problem 4: Transform lookup failures**
```bash
# Error in logs:
[ERROR] [tf2_buffer]: Could not transform from 'base_scan' to 'map'

# Check transform chain
$ ros2 run tf2_ros tf2_echo map base_scan
[ERROR] Lookup would require extrapolation into the past

# Debug transform timestamps
$ ros2 topic echo /tf | head -20
header:
  stamp: {sec: 1000, nanosec: 500000000}  # Check if timestamps are current

# Solution: Synchronize clocks, check transform_timeout parameters
transform_timeout: 0.1  # Increase timeout in nav2 config
```

**Problem 5: Robot gets stuck in corners**
```bash
# Check costmap configuration
$ ros2 topic echo /local_costmap/costmap --once
# Look for excessive inflation around obstacles

# Visualize costmap in RViz
# Add -> By Topic -> /local_costmap/costmap -> Map

# Tune parameters:
inflation_radius: 0.15      # Reduce if robot can't fit through gaps
cost_scaling_factor: 10.0   # Increase to make robot more cautious
```

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
