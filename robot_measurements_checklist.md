### **Robot Physical Measurements Checklist**

#### **1. Base Link (Robot Chassis)**
These measurements define the main body of your robot.

* **Length:** The front-to-back dimension of the chassis.
* **Width:** The side-to-side dimension of the chassis.
* **Height:** The top-to-bottom dimension of the chassis.
* **Mass:** The total mass of the robot's chassis. If you don't have a scale, you can get a good estimate.

#### **2. Differential Drive Wheels**
These are critical for the robot's kinematics, which is how the robot moves.

* **Wheel Radius:** The distance from the center of the wheel to its edge.
* **Wheel Width:** The thickness of the wheel.
* **Track Width:** The distance between the centers of the two wheels.
* **Wheel Position:** The X, Y, and Z coordinates of the wheel's center relative to the center of your robot's base. It's usually easiest to define the robot's center as (0, 0, 0).

#### **3. LiDAR Sensor**
These measurements tell the robot where its "eyes" are located.

* **Position:** The X, Y, and Z coordinates of the LiDAR's center relative to the center of the robot's base.
* **Orientation:** The roll, pitch, and yaw angles of the LiDAR sensor relative to the robot's base.

#### **4. IMU Sensor**
The IMU helps the robot understand its orientation and acceleration.

* **Position:** The X, Y, and Z coordinates of the IMU's center relative to the center of the robot's base.
* **Orientation:** The roll, pitch, and yaw angles of the IMU sensor relative to the robot's base.
