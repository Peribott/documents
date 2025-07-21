# ROS2 Drivers for Custom Robotic Platform

## Overview
This document describes the ROS2 driver implementation for a custom robotic platform featuring STM32 microcontroller, BLDC motors, 169mm diameter wheels, and ultrasonic sensors.

## Hardware Specifications

### Platform Components
- **Microcontroller**: STM32 (ARM Cortex-M series)
- **Motors**: BLDC (Brushless DC) Motors
- **Wheels**: 169mm diameter wheels
- **Sensors**: Ultrasonic distance sensors
- **Communication**: UART/USB for ROS2 interface

### Key Parameters
- Wheel diameter: 169mm (0.169m)
- Wheel circumference: π × 0.169m ≈ 0.531m
- Motor type: Brushless DC (3-phase)
- Sensor range: Ultrasonic (typically 2cm - 4m)

## System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   ROS2 Node     │◄──►│  STM32 MCU       │◄──►│  BLDC Motors    │
│  (Linux Host)   │    │  (Firmware)      │    │  + Encoders     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         │                       │               ┌───────▼────────┐
         │                       │               │   Wheels        │
         │                       │               │   (169mm ⌀)    │
         │                       │               └────────────────┘
         │                       │
         │                       ▼
         │               ┌──────────────────┐
         └──────────────►│ Ultrasonic       │
                         │ Sensors          │
                         └──────────────────┘
```

## ROS2 Package Structure

```
custom_robot_drivers/
├── package.xml
├── CMakeLists.txt
├── setup.py
├── src/
│   └── custom_robot_drivers/
│       ├── __init__.py
│       ├── motor_controller.py
│       ├── ultrasonic_sensor.py
│       ├── robot_base.py
│       └── hardware_interface.py
├── launch/
│   ├── robot_bringup.launch.py
│   └── sensors.launch.py
├── config/
│   ├── robot_params.yaml
│   └── motor_config.yaml
├── msg/
│   ├── MotorStatus.msg
│   └── UltrasonicReading.msg
├── srv/
│   └── MotorCalibration.srv
└── urdf/
    └── robot_description.urdf
```

## Message Definitions

### MotorStatus.msg
```
# Motor status message
Header header
uint8 motor_id
float32 rpm
float32 current_consumption
float32 temperature
uint8 error_code
bool is_enabled
```

### UltrasonicReading.msg
```
# Ultrasonic sensor reading
Header header
uint8 sensor_id
float32 range
float32 field_of_view
float32 min_range
float32 max_range
```

## Service Definitions

### MotorCalibration.srv
```
# Motor calibration service
uint8 motor_id
bool enable_calibration
---
bool success
string message
```

## Node Implementation

### 1. Motor Controller Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from custom_robot_drivers.msg import MotorStatus
import serial
import struct
import math

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_diameter', 0.169)  # 169mm
        self.declare_parameter('wheel_base', 0.3)        # 30cm between wheels
        self.declare_parameter('max_rpm', 1000)
        
        # Serial communication
        self.serial_port = serial.Serial(
            self.get_parameter('serial_port').value,
            self.get_parameter('baud_rate').value,
            timeout=1.0
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.motor_status_pub = self.create_publisher(MotorStatus, 'motor_status', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Timers
        self.create_timer(0.1, self.read_motor_feedback)  # 10Hz
        
        self.get_logger().info('Motor Controller Node Started')
    
    def cmd_vel_callback(self, msg):
        """Convert Twist to motor commands"""
        linear_vel = msg.linear.x  # m/s
        angular_vel = msg.angular.z  # rad/s
        
        wheel_radius = self.get_parameter('wheel_diameter').value / 2
        wheel_base = self.get_parameter('wheel_base').value
        
        # Differential drive kinematics
        left_vel = linear_vel - (angular_vel * wheel_base / 2)
        right_vel = linear_vel + (angular_vel * wheel_base / 2)
        
        # Convert to RPM
        left_rpm = (left_vel / (wheel_radius * 2 * math.pi)) * 60
        right_rpm = (right_vel / (wheel_radius * 2 * math.pi)) * 60
        
        self.send_motor_command(left_rpm, right_rpm)
    
    def send_motor_command(self, left_rpm, right_rpm):
        """Send motor command to STM32"""
        # Protocol: [HEADER][CMD][LEFT_RPM][RIGHT_RPM][CHECKSUM]
        header = 0xAA
        cmd = 0x01  # Motor command
        
        data = struct.pack('<BBff', header, cmd, left_rpm, right_rpm)
        checksum = sum(data) & 0xFF
        
        packet = data + struct.pack('<B', checksum)
        self.serial_port.write(packet)
```

### 2. Ultrasonic Sensor Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from custom_robot_drivers.msg import UltrasonicReading
import serial
import struct

class UltrasonicSensor(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('sensor_count', 4)
        self.declare_parameter('min_range', 0.02)  # 2cm
        self.declare_parameter('max_range', 4.0)   # 4m
        self.declare_parameter('field_of_view', 0.26)  # 15 degrees
        
        # Publishers
        self.range_pubs = []
        self.ultrasonic_pub = self.create_publisher(UltrasonicReading, 'ultrasonic', 10)
        
        sensor_count = self.get_parameter('sensor_count').value
        for i in range(sensor_count):
            pub = self.create_publisher(Range, f'ultrasonic_{i}', 10)
            self.range_pubs.append(pub)
        
        # Serial communication
        self.serial_port = serial.Serial(
            self.get_parameter('serial_port').value,
            self.get_parameter('baud_rate').value,
            timeout=1.0
        )
        
        # Timer for reading sensors
        self.create_timer(0.05, self.read_sensors)  # 20Hz
        
        self.get_logger().info('Ultrasonic Sensor Node Started')
    
    def read_sensors(self):
        """Read ultrasonic sensor data from STM32"""
        try:
            if self.serial_port.in_waiting > 0:
                # Protocol: [HEADER][SENSOR_COUNT][RANGES...][CHECKSUM]
                data = self.serial_port.read(20)  # Adjust based on protocol
                
                if len(data) >= 10:  # Minimum packet size
                    header = data[0]
                    if header == 0xBB:  # Ultrasonic data header
                        sensor_count = data[1]
                        ranges = struct.unpack('<' + 'f' * sensor_count, 
                                             data[2:2 + sensor_count * 4])
                        
                        self.publish_sensor_data(ranges)
        
        except Exception as e:
            self.get_logger().error(f'Error reading ultrasonic data: {e}')
    
    def publish_sensor_data(self, ranges):
        """Publish sensor data as ROS2 messages"""
        current_time = self.get_clock().now().to_msg()
        
        for i, range_val in enumerate(ranges):
            # Standard Range message
            range_msg = Range()
            range_msg.header.stamp = current_time
            range_msg.header.frame_id = f'ultrasonic_{i}'
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = self.get_parameter('field_of_view').value
            range_msg.min_range = self.get_parameter('min_range').value
            range_msg.max_range = self.get_parameter('max_range').value
            range_msg.range = range_val
            
            self.range_pubs[i].publish(range_msg)
            
            # Custom UltrasonicReading message
            ultrasonic_msg = UltrasonicReading()
            ultrasonic_msg.header = range_msg.header
            ultrasonic_msg.sensor_id = i
            ultrasonic_msg.range = range_val
            ultrasonic_msg.field_of_view = range_msg.field_of_view
            ultrasonic_msg.min_range = range_msg.min_range
            ultrasonic_msg.max_range = range_msg.max_range
            
            self.ultrasonic_pub.publish(ultrasonic_msg)
```

## STM32 Firmware Interface

### Communication Protocol

#### Motor Control Commands
```c
// Motor command packet structure
typedef struct {
    uint8_t header;      // 0xAA
    uint8_t cmd;         // Command type
    float left_rpm;      // Left motor RPM
    float right_rpm;     // Right motor RPM
    uint8_t checksum;    // Simple checksum
} motor_cmd_t;

// Motor feedback packet
typedef struct {
    uint8_t header;      // 0xCC
    uint8_t motor_id;    // Motor identifier
    float actual_rpm;    // Actual RPM
    float current;       // Current consumption (A)
    float temperature;   // Motor temperature (°C)
    uint8_t error_code;  // Error status
    uint8_t checksum;
} motor_feedback_t;
```

#### Ultrasonic Sensor Data
```c
// Ultrasonic data packet
typedef struct {
    uint8_t header;          // 0xBB
    uint8_t sensor_count;    // Number of sensors
    float ranges[4];         // Range readings (m)
    uint8_t checksum;
} ultrasonic_data_t;
```

### STM32 Main Functions
```c
// Motor control
void bldc_set_speed(uint8_t motor_id, float rpm);
float bldc_get_speed(uint8_t motor_id);
float bldc_get_current(uint8_t motor_id);

// Ultrasonic sensors
float ultrasonic_read_distance(uint8_t sensor_id);
void ultrasonic_trigger_all(void);

// Communication
void uart_send_motor_feedback(void);
void uart_send_ultrasonic_data(void);
void uart_process_commands(void);
```

## Configuration Files

### robot_params.yaml
```yaml
robot_controller:
  ros__parameters:
    # Physical parameters
    wheel_diameter: 0.169        # meters
    wheel_base: 0.3              # meters
    max_linear_velocity: 2.0     # m/s
    max_angular_velocity: 3.14   # rad/s
    
    # Communication
    serial_port: "/dev/ttyUSB0"
    baud_rate: 115200
    
    # Control parameters
    pid_kp: 1.0
    pid_ki: 0.1
    pid_kd: 0.05
    
    # Safety limits
    max_motor_current: 5.0       # Amperes
    max_motor_temperature: 80.0  # Celsius
```

### motor_config.yaml
```yaml
motor_controller:
  ros__parameters:
    # Motor specifications
    motor_count: 2
    max_rpm: 1000
    rated_voltage: 24.0          # Volts
    rated_current: 3.0           # Amperes
    
    # Encoder settings
    encoder_resolution: 1024     # PPR
    gear_ratio: 10.0            # Motor gear reduction
    
    # BLDC controller settings
    pwm_frequency: 20000         # Hz
    commutation_method: "hall"   # hall, encoder, sensorless
```

## Launch Files

### robot_bringup.launch.py
```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        
        # Motor controller node
        Node(
            package='custom_robot_drivers',
            executable='motor_controller',
            name='motor_controller',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'serial_port': LaunchConfiguration('serial_port')},
                'config/robot_params.yaml'
            ],
            output='screen'
        ),
        
        # Ultrasonic sensor node
        Node(
            package='custom_robot_drivers',
            executable='ultrasonic_sensor',
            name='ultrasonic_sensor',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                'config/robot_params.yaml'
            ],
            output='screen'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': 'urdf/robot_description.urdf'}]
        ),
    ])
```

## URDF Description

### robot_description.urdf
```xml
<?xml version="1.0"?>
<robot name="custom_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Wheels -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.0845" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>
  
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.0845" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>
  
  <!-- Wheel joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  
  <!-- Ultrasonic sensors -->
  <link name="ultrasonic_front">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.01"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="ultrasonic_front_joint" type="fixed">
    <parent link="base_link"/>
    <child link="ultrasonic_front"/>
    <origin xyz="0.2 0 0.02" rpy="0 0 0"/>
  </joint>
</robot>
```

## Package Configuration

### package.xml
```xml
<?xml version="1.0"?>
<package format="3">
  <name>custom_robot_drivers</name>
  <version>1.0.0</version>
  <description>ROS2 drivers for custom robotic platform</description>
  
  <maintainer email="user@example.com">Robot Developer</maintainer>
  <license>Apache-2.0</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>robot_state_publisher</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### setup.py
```python
from setuptools import setup
import os
from glob import glob

package_name = 'custom_robot_drivers'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Developer',
    maintainer_email='user@example.com',
    description='ROS2 drivers for custom robotic platform',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = custom_robot_drivers.motor_controller:main',
            'ultrasonic_sensor = custom_robot_drivers.ultrasonic_sensor:main',
            'robot_base = custom_robot_drivers.robot_base:main',
        ],
    },
)
```

## Building and Running

### Build Commands
```bash
# Create workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clone or copy the package
# Build the workspace
cd ~/robot_ws
colcon build --packages-select custom_robot_drivers

# Source the workspace
source install/setup.bash
```

### Launch Commands
```bash
# Launch the complete robot system
ros2 launch custom_robot_drivers robot_bringup.launch.py

# Launch individual nodes
ros2 run custom_robot_drivers motor_controller
ros2 run custom_robot_drivers ultrasonic_sensor

# Test motor commands
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'

# Monitor sensor data
ros2 topic echo /ultrasonic
ros2 topic echo /motor_status
```

## Testing and Calibration

### Motor Calibration
1. Ensure motors are properly connected and powered
2. Run calibration service:
   ```bash
   ros2 service call /motor_calibration custom_robot_drivers/srv/MotorCalibration '{motor_id: 0, enable_calibration: true}'
   ```
3. Verify encoder feedback and motor direction

### Sensor Validation
1. Test ultrasonic sensors with known distances
2. Verify range accuracy and repeatability
3. Check sensor mounting and field of view

### Safety Features
- Motor current monitoring
- Temperature protection
- Emergency stop capability
- Velocity limiting
- Sensor range validation

## Troubleshooting

### Common Issues
1. **Serial Communication Errors**
   - Check serial port permissions: `sudo usermod -a -G dialout $USER`
   - Verify baud rate settings
   - Check cable connections

2. **Motor Not Responding**
   - Verify power supply voltage (24V for BLDC)
   - Check motor controller connections
   - Validate PWM signals

3. **Inaccurate Odometry**
   - Calibrate wheel diameter parameter
   - Check encoder resolution settings
   - Verify gear ratio configuration

4. **Ultrasonic Sensor Issues**
   - Check sensor mounting and orientation
   - Validate range limits
   - Test in different environments

### Debug Commands
```bash
# Check node status
ros2 node list
ros2 node info /motor_controller

# Monitor topics
ros2 topic list
ros2 topic hz /cmd_vel
ros2 topic echo /odom

# Check parameters
ros2 param list /motor_controller
ros2 param get /motor_controller wheel_diameter
```

## Future Enhancements

1. **Additional Sensors**
   - IMU integration for better odometry
   - Camera for visual navigation
   - LIDAR for advanced mapping

2. **Advanced Features**
   - SLAM capabilities
   - Autonomous navigation
   - Web-based control interface
   - Remote monitoring dashboard

3. **Performance Optimizations**
   - Real-time control loop
   - Advanced motor control algorithms
   - Sensor fusion techniques
   - Machine learning integration

---

*This documentation provides a comprehensive guide for implementing ROS2 drivers for your custom robotic platform. Adjust parameters and configurations based on your specific hardware setup and requirements.*


## References and Guides

### ROS2 Documentation and Tutorials
- **Official ROS2 Documentation**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
- **ROS2 Tutorials**: [https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)
- **Creating Custom ROS2 Messages**: [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- **ROS2 Launch System**: [https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- **ROS2 Parameters**: [https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)

### Hardware Interface and Serial Communication
- **PySerial Documentation**: [https://pyserial.readthedocs.io/en/latest/](https://pyserial.readthedocs.io/en/latest/)
- **ROS2 Hardware Interface**: [https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html)
- **ROS2 Control Framework**: [https://control.ros.org/humble/index.html](https://control.ros.org/humble/index.html)

### STM32 Development
- **STM32CubeMX**: [https://www.st.com/en/development-tools/stm32cubemx.html](https://www.st.com/en/development-tools/stm32cubemx.html)
- **STM32 HAL Documentation**: [https://www.st.com/resource/en/user_manual/um1725-description-of-stm32f4-hal-and-lowlayer-drivers-stmicroelectronics.pdf](https://www.st.com/resource/en/user_manual/um1725-description-of-stm32f4-hal-and-lowlayer-drivers-stmicroelectronics.pdf)
- **STM32 UART Communication**: [https://wiki.st.com/stm32mcu/wiki/Getting_started_with_UART](https://wiki.st.com/stm32mcu/wiki/Getting_started_with_UART)
- **STM32 Timer PWM**: [https://wiki.st.com/stm32mcu/wiki/Getting_started_with_PWM](https://wiki.st.com/stm32mcu/wiki/Getting_started_with_PWM)

### BLDC Motor Control
- **BLDC Motor Control Fundamentals**: [https://www.ti.com/lit/an/spra588/spra588.pdf](https://www.ti.com/lit/an/spra588/spra588.pdf)
- **Sensored BLDC Motor Control**: [https://www.microchip.com/en-us/application-notes/an885](https://www.microchip.com/en-us/application-notes/an885)
- **STM32 Motor Control SDK**: [https://www.st.com/en/embedded-software/x-cube-mcsdk.html](https://www.st.com/en/embedded-software/x-cube-mcsdk.html)
- **FOC (Field Oriented Control) Theory**: [https://www.ti.com/lit/an/spra588/spra588.pdf](https://www.ti.com/lit/an/spra588/spra588.pdf)

### Differential Drive and Kinematics
- **Differential Drive Robot Kinematics**: [https://automaticaddison.com/the-ultimate-guide-to-differential-drive-kinematics/](https://automaticaddison.com/the-ultimate-guide-to-differential-drive-kinematics/)
- **Mobile Robot Kinematics**: [http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf](http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)
- **ROS Navigation Stack**: [https://navigation.ros.org/](https://navigation.ros.org/)

### Sensors and Perception
- **Ultrasonic Sensor Interfacing**: [https://www.arduino.cc/en/Tutorial/BuiltInExamples/Ping](https://www.arduino.cc/en/Tutorial/BuiltInExamples/Ping)
- **ROS2 Sensor Msgs**: [https://docs.ros2.org/latest/api/sensor_msgs/](https://docs.ros2.org/latest/api/sensor_msgs/)
- **TF2 (Transform Library)**: [https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)

### URDF and Robot Description
- **URDF Tutorials**: [https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- **Robot State Publisher**: [https://github.com/ros/robot_state_publisher](https://github.com/ros/robot_state_publisher)
- **Joint State Publisher**: [https://github.com/ros/joint_state_publisher](https://github.com/ros/joint_state_publisher)

### Testing and Debugging
- **ROS2 Testing Guide**: [https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)
- **rqt Tools**: [https://docs.ros.org/en/humble/Concepts/About-RQt.html](https://docs.ros.org/en/humble/Concepts/About-RQt.html)
- **ROS2 Bag Recording**: [https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

### Advanced Topics
- **ROS2 Real-time**: [https://docs.ros.org/en/humble/Tutorials/Miscellaneous/Building-Realtime-rt_preempt-kernel-for-ROS-2.html](https://docs.ros.org/en/humble/Tutorials/Miscellaneous/Building-Realtime-rt_preempt-kernel-for-ROS-2.html)
- **ROS2 Security**: [https://docs.ros.org/en/humble/Tutorials/Advanced/Security/Security-Main.html](https://docs.ros.org/en/humble/Tutorials/Advanced/Security/Security-Main.html)
- **MoveIt2 Motion Planning**: [https://moveit.ros.org/](https://moveit.ros.org/)
- **Nav2 Navigation Framework**: [https://navigation.ros.org/](https://navigation.ros.org/)

### Community and Support
- **ROS Discourse Forum**: [https://discourse.ros.org/](https://discourse.ros.org/)
- **ROS Answers**: [https://answers.ros.org/](https://answers.ros.org/)
- **GitHub ROS2 Organization**: [https://github.com/ros2](https://github.com/ros2)
- **STM32 Community**: [https://community.st.com/](https://community.st.com/)

### Example Repositories
- **ROS2 Examples**: [https://github.com/ros2/examples](https://github.com/ros2/examples)
- **ROS2 Demo Nodes**: [https://github.com/ros2/demos](https://github.com/ros2/demos)
- **Differential Drive Robot Examples**: [https://github.com/joshnewans/diffdrive_arduino](https://github.com/joshnewans/diffdrive_arduino)
- **STM32 ROS Communication**: [https://github.com/grassjelly/linorobot2_hardware](https://github.com/grassjelly/linorobot2_hardware)

### Development Tools
- **VS Code ROS Extension**: [https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)
- **STM32CubeIDE**: [https://www.st.com/en/development-tools/stm32cubeide.html](https://www.st.com/en/development-tools/stm32cubeide.html)
- **PlatformIO**: [https://platformio.org/](https://platformio.org/)
- **Colcon Build Tool**: [https://colcon.readthedocs.io/en/released/](https://colcon.readthedocs.io/en/released/)

### Standards and Protocols
- **REP (ROS Enhancement Proposals)**: [https://www.ros.org/reps/rep-0000.html](https://www.ros.org/reps/rep-0000.html)
- **ROS2 Design Documents**: [https://design.ros2.org/](https://design.ros2.org/)
- **DDS (Data Distribution Service)**: [https://www.omg.org/omg-dds-portal/](https://www.omg.org/omg-dds-portal/)

---

*Last updated: July 2025*
