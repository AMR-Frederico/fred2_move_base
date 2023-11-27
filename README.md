# Fred - Move Base

**note: Package for ROS 2 - C++/Python (CMake) based package**

The move_base_package is a comprehensive package designed to control various aspects of the robot's behavior. Here's an overview of its key functionalities:

- **LED Strip Control:** 
The package efficiently manages the LED strip on the robot, dynamically adjusting its state based on the current state of the robot.

- **Odometry Calculation:** Utilizing encoder and IMU (Inertial Measurement Unit) data, the package accurately calculates odometry, providing essential information about the robot's position and orientation.

- **Joystick Commands:** The package seamlessly integrates with joystick commands, offering both manual and autonomous control modes. Users can command the robot through the joystick, including emergency stop functionality and velocity control.

- **Safety Determination:** A critical feature of the move_base_package is its capability to assess the safety of robot movements. It considers both joystick commands, such as emergency stop signals, and ultrasonic readings to determine whether it is secure to initiate robot movements.

----

## Led manager node
Determinate the LED strip's color basead on the robot state. Responsable for the waypoint sinalization. 

**Type:** Python 

**Name:** led_manager

**Namespace:** safe_twist

### Topics
**Publishers:**

|          Name           |       Type      | 
|:----------------------- |:--------------- |
|  `/cmd/led_strip/color` | `std_msgs/Int16`| 

<br>

**Subscribers:**
|             Name                 |             Type            |
| :--------------------------------| :-------------------------  |
|`safety/abort/colision_detection` |       `std_msgs/Bool`       |
|   `safety/abort/user_command`    |       `std_msgs/Bool`       |
|`machine_states/main/robot_status`|     `std_msgs/Int16`        |
|    `goal_manager/goal/current`   | `geometry_msgs/PoseStamped` |
|   `goal_manager/goal/reached`    |       `std_msgs/Bool`       |

 <br>

### Run 
**Default:**

```
ros2 run fred2_move_base led_manager.py
```

**Enable debug:**
```
ros2 run fred2_move_base led_manager.py --debug
```