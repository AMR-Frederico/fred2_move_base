# Fred - Move Base

**note: Package for ROS 2 - C++/Python (CMake) based package**

The move_base_package is a comprehensive package designed to control various aspects of the robot's behavior. Here's an overview of its key functionalities:

- **LED Strip Control:** 
The package efficiently manages the LED strip on the robot, dynamically adjusting its state based on the current state of the robot.

- **Odometry Calculation:** Utilizing encoder and IMU (Inertial Measurement Unit) data, the package accurately calculates odometry, providing essential information about the robot's position and orientation.

- **Joystick Commands:** The package seamlessly integrates with joystick commands, offering both manual and autonomous control modes. Users can command the robot through the joystick, including emergency stop functionality and velocity control.

- **Safety Determination:** A critical feature of the move_base_package is its capability to assess the safety of robot movements. It considers both joystick commands, such as emergency stop signals, and ultrasonic readings to determine whether it is secure to initiate robot movements.

----

## Installation 

**1. Clone the repository into your ROS2 workspace:**

```bash
cd ros2_ws/src
git clone https://github.com/AMR-Frederico/fred2_move_base.git
```

**2. Build the package:**

```bash
cd ros2_ws
colcon build
```

**3. Install the `transforms3d` library:**

```bash
pip install transforms3d
```

----

## Usage

**Launch the package:**

```bash
ros2 launch fred2_move_base move_base_launch.yaml
```

**Joystick commands:**

- Use the left analog stick for linear velocity control.
- Use the right analog stick for angular velocity control.
- Press the circle button to reset odometry.
- Press the triangle button to switch between different modes.
- Press the X button to stop and block/release the robot

---

## Led manager node
Determinate the LED strip's color basead on the robot state. Responsable for the waypoint sinalization. 

**Type:** `python` 

**Name:** `led_manager`

**Namespace:** `move_base`

### Topics
**Publishers:**

|          Name                |       Type      | 
|:-----------------------------|:--------------- |
|  `/cmd/led_strip/color`      | `std_msgs/Int16`| 
| `/cmd/led_strip/debug/color` | `std_msgs/Int16`|

<br>
 
**Subscribers:**
|             Name                 |             Type            |
| :--------------------------------| :-------------------------  |
|`safety/abort/colision_detection` |       `std_msgs/Bool`       |
|   `safety/abort/user_command`    |       `std_msgs/Bool`       |
|   `safety/ultrasonic/disabled`   |       `std_msgs/Bool`       |
|    `joy/controller/connected`    |       `std_msgs/Bool`       |
|  `machine_states/robot_status`   |      `std_msgs/Int16`       |
|    `goal_manager/goal/current`   | `geometry_msgs/PoseStamped` |
|           `/odom/reset`          |       `std_msgs/Bool`       |

 <br>

### Parameters: 

- `WHITE`: Index to indicate white color on the LED strip
- `BLUE`: Index to indicate blue color on the LED strip
- `YELLOW`: Index to indicate yellow color on the LED strip
- `PINK`: Index to indicate pink color on the LED strip
- `GREEN`: Index to indicate green color on the LED strip
- `ORANGE`: Index to indicate orange color on the LED strip
- `RED`: Index to indicate red color on the LED strip
- `CYAN`: Index to indicate cyan color on the LED strip
- `PURPLE`: Index to indicate purple color on the LED strip 
- `LIGHT GREEN`: Index to indicate light green color on the LED strip
- `BLACK`: Index to turn off the LED strip

- `WAYPOINT_GOAL`: Index to indicate goals points that the robot must signal
- `GHOST_GOAL`: Index to indicate ghost goals


### Run 
**Default:**

```
ros2 run fred2_move_base led_manager.py
```

**Enable debug:**
```
ros2 run fred2_move_base led_manager.py --debug
```

----- 
## Safe Twist

The node is responsible for managing the velocity commands of a robot based on safety constraints, including ultrasonic sensor readings and user commands.


**Type:** `python` 

**Name:** `safe_twist`

**Namespace:** `move_base`

### Topics
**Publishers:**

|               Name                |          Type        | 
|:-----------------------           |:---------------------|
|         `/cmd_vel/safe`           | `geometry_msgs/Twist`| 
|    `safety/abort/user_command`    |   `std_msgs/Bool`    |
|   `safety/abort/colision_alert`   |   `std_msgs/Bool`    |
|    `safety/ultrasonic/disabled`   |   `std_msgs/Bool`    |
|           `robot_safety`          |   `std_msgs/Bool`    |

<br>

**Subscribers:**
|             Name                 |             Type            |
| :--------------------------------| :-------------------------  |
| `/sensor/range/ultrasonic/right` |     `std_msgs/Float32`      |
| `/sensor/range/ultrasonic/left`  |     `std_msgs/Float32`      |
| `/sensor/range/ultrasonic/back`  |     `std_msgs/Float32`      |
|      `/odometry/filtered`        |     `nav_msgs/Odometry`     |
|             `/odom`              |     `nav_msgs/Odometry`     |
|            `/cmd_vel`            |    `geometry_msgs/Twist`    |
|   `/joy/controler/ps4/break`     |       `std_msgs/Bool`       |

 <br>

### Parameters 

- `SAFE_DISTANCE`: Minimum safe distance (in cm) that the robot can be from an object
 
- `MOTOR_BRAKE_FACTOR`: Brake factor for stop the robot as quickly as possible
 
- `MAX_LINEAR_SPEED`: Maximum linear speed of the robot

- `MAX_ANGULAR_SPEED`: Maximum angular of the robot

- `DISABLE_ULTRASONICS`: Parameter for disable the ultrasonics reading, therefore, disable detection alert



### Run 
**Default:**

```
ros2 run fred2_move_base safe_twist.py
```

**Enable debug:**
```
ros2 run fred2_move_base safe_twist.py --debug
```

---

## Joystick interface

The Joy Interface is a node that interfaces with a joystick (e.g., PS4 controller) to control a mobile robot. It receives the joystick commands, sent by a microcontroller and translates it into velocity commands for the robot, allowing for manual control and additional functionalities.

**Type:** `python` 

**Name:** `joy_esp_interface`

**Namespace:** `move_base`

### Topics
**Publishers:**

|               Name                     |          Type         | 
|:-----------------------                |:--------------------- |
|`/cmd_vel`                              |  `geometry_msgs/Twist |
|`/odom/reset`                           |  `std_msgs/Bool`      |
|`/joy/machine_states/switch_mode`       |  `std_msgs/Bool`      |
| `/goal_manager/goal/mission_completed` |  `std_msgs/Bool`      |



<br>

**Subscribers:**
|             Name                       |             Type            |
| :--------------------------------------| :-------------------------  |
|`/joy/controler/ps4/cmd_vel/linear`     |      `std_msgs/Int16`       |    
|`/joy/controler/ps4/cmd_vel/angular`    |      `std_msgs/Int16`       |
|`/joy/controler/ps4/circle`             |      `std_msgs/Int16`       |
|`/joy/controler/ps4/triangle`           |      `std_msgs/Int16`       |
|`/machine_state/robot_state`            |      `std_msgs/Bool`        |




 <br>

### Pararams

- `MAX_SPEED_JOY_LINEAR`: Maximum linear speed of the robot.
- `MAX_SPEED_JOY_ANGULAR`: Maximum angular speed of the robot.
- `MAX_VALUE_CONTROLLERr`: Maximum value of analog joystick input.
- `DRIFT_ANALOG_TOLERANCE`: Minimum threshold for joystick input to be considered.


### Run 
**Default:**

```
ros2 run fred2_move_base joy_esp_interface.py
```

**Enable debug:**
```
ros2 run fred2_move_base joy_esp_interface.py --debug
```
---

## Odometry
The Odometry Node is responsible for calculating and publishing odometry information for a mobile robot using wheel encoder ticks and IMU data, with automatic reset of odometry when requested. 

**Type:** `python` 

**Name:** `odometry`

**Namespace:** `move_base`

### Topics
**Publishers:**

|               Name                 |          Type        | 
|:-----------------------            |:---------------------|
|`/power/status/distance/ticks/right` |   `std_msgs/Int32`   | 
|`/power/status/distance/ticks/left` |   `std_msgs/Int32`   |
|       `/sensor/orientation/imu`    |   `sensor_msgs/Imu`  |
|             `/odom/reset`          |   `std_msgs/Bool`    |

<br>

**Subscribers:**
|             Name                 |             Type            |
| :--------------------------------| :-------------------------  |
|             `/odom`              |     `nav_msgs/Odometry`     |

 <br>

### Pararams
- `WHEELS_TRACK`: Distance (in m) between wheels 
- `WHEELS_RADIUS`: Radius of the wheel in meters.
- `TICKS_PER_REVOLUTION`: Ticks per turn for the wheel encoder.
- `BASE_LINK_OFFSET`: Distance (in meters) between the `base_footprint` and `base_link` 


### Run 
**Default:**

```
ros2 run fred2_move_base ticks2odom.py
```

**Enable debug:**
```
ros2 run fred2_move_base ticks2odom.py --debug
```

**Publish `odom`, `base_link` and `base_footprint` TF:**
```
ros2 run fred2_move_base ticks2odom.py --publish-tf
```
---- 

## Launch

**Considering `Robot Localization` for odometry and `Robot Descriptor` for publish the TFs:**
```
ros2 launch fred2_move_base move_base.launch.py
```

**Considering `Move Base Odometry` to publish odom and TF:**
```
ros2 launch fred2_move_base move_base_with_odom_tf.launch.py
```