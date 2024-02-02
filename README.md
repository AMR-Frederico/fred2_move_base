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

### Launch

**Considering `Robot Localization` for odometry and `Robot Descriptor` for publish the TFs:**
```
ros2 launch fred2_move_base move_base.launch.py
```

**Considering `Move Base Odometry` to publish odom and TF:**
```
ros2 launch fred2_move_base move_base_with_odom_tf.launch.py
```

### Joystick commands

- Use the `left analog` stick for *linear velocity contro*l.

- Use the `right analog` stick for *angular velocity control*.

- Press the `circle button` to *reset odometry*.

- Press the `triangle button` to *switch* between different *modes*.

- Press the `X button` to *stop* and block/release the robot

---

## Led manager node
Determinate the LED strip's color basead on the robot state. Responsable for the waypoint sinalization. 

**Type:** `python` 

**Name:** `led_manager`

**Namespace:** `move_base`

### Topics

**Publishers:**

- `/cmd/led_strip/color`(*std_msgs/Int16*): Colors that indicates the main state of the robot

- `/cmd/led_strip/debug/color` (*std_msgs/Int16*): Color for debug the robot status, like joy connectivity or alert of colision detection. 

<br>
 
**Subscribers:**

- `safety/abort/colision_detection` (*std_msgs/Bool*): Returns `True` if a obstacle is detected, otherwise is `False`

- `safety/abort/user_command` (*std_msgs/Bool*): Returns `True` if the user pressed `X`, stopping the robot, and `False` if the user press `X` again to release the robot

- `safety/ultrasonic/disabled` (*std_msgs/Bool*): Publish `True` if the ultrasonics were desabled, otherwise is `False`

- `joy/controller/connected` (*std_msgs/Bool*): Joy status connectivity. 

- `machine_states/robot_state` (*std_msgs/Int16*): Robot state from the main machine states

- `goal_manager/goal/current` (*geometry_msgs/PoseStamped*): Robot current goal

- `/odom/reset` (*std_msgs/Bool*): Reset odometry and tobot state

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

- `/cmd_vel/safe` (*geometry_msgs/Twist*): Safe velocity command for the robot

- `safety/abort/user_command` (*std_msgs/Bool*): Topic for debug, to inform a user request command to stop the robot
- `safety/abort/colision_alert` (*std_msgs/Bool*): Topic for debug, to inform when there is a collision alert 
- `safety/ultrasonic/disabled` (*std_msgs/Bool*): Topic for debug, to inform when the ultrsonics are disable 
- `robot_safety` (*std_msgs/Bool*): Robot safety status, returns `True` when the robot is safe, and `False` when the robot is blocked by `safe twist node`

<br>

**Subscribers:**

- `/sensor/range/ultrasonic/right` (*std_msgs/Float32*): Ultrasonic right data 

- `/sensor/range/ultrasonic/left` (*std_msgs/Float32`*): Ultrasonic left data
- `/sensor/range/ultrasonic/back` (*std_msgs/Float32*): Ultrsonic back data 
- `/odometry/filtered` (*nav_msgs/Odometry*): Odometry from `Robot Localization `
- `/odom` (*nav_msgs/Odometry*): Odometry from `Move Base`
- `/cmd_vel` (*geometry_msgs/Twist*): Velocity commands from other nodes 
- `/joy/controler/ps4/break` (*std_msgs/Bool*): User stop command by the joystick 


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
- `/cmd_vel` (*geometry_msgs/Twist*): Velocity command 

- `/odom/reset` (*std_msgs/Bool*): Command for reset the odometry and reset the robot state

- `/joy/machine_states/switch_mode` (*std_msgs/Bool*): Switch between `MANUAL` and `AUTONOMOUS` mode.

- `/goal_manager/goal/mission_completed` (*std_msgs/Bool*): Publish false when received a command for reset the odometry



<br>

**Subscribers:**
- `/joy/controler/ps4/cmd_vel/linear` (*std_msgs/Int16*): Linear velocity from the analog joy control

- `/joy/controler/ps4/cmd_vel/angular` (*std_msgs/Int16*): Angular velocity from the analog joy control

- `/joy/controler/ps4/circle` (*std_msgs/Int16*): Get the circle button command 

- `/joy/controler/ps4/triangle` (*std_msgs/Int16*): Get the triangle button command 

- `/machine_state/robot_state` (*std_msgs/Bool*): Get the robot state




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

- `/power/status/distance/ticks/right` (*std_msgs/Int32*): Ticks data from the right encoders
- `/power/status/distance/ticks/left` (*std_msgs/Int32*): Ticks data from left encoders 
- `/sensor/orientation/imu` (*sensor_msgs/Imu*): Imu raw data
- `/odom/reset` (*std_msgs/Bool*): Command for reset the odometry

<br>

**Subscribers:**
- `/odom` (*nav_msgs/Odometry*): Odometry (without kalman filter)

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

