import fred2_move_base.scripts.qos as qos
import transforms3d as tf3d     # angle manipulaton 

from fred2_move_base.scripts.safe_twist import SafeTwistNode
from fred2_move_base.scripts.led_manager import LedManagerNode
from fred2_move_base.scripts.joy_esp_interface import JoyInterfaceNode

from rclpy.node import Node 

from std_msgs.msg import Int16, Bool, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Imu
from nav_msgs.msg import Odometry

commum_qos_profile  = qos.general_configuration()
imu_qos_profile = qos.imu_configuration()


########################################################
# --------------- SAFE TWIST
########################################################

def safe_config(node: Node): 

    global commum_qos_profile


    # --------------- Ultrasonic sensors (from the firmware)
    node.create_subscription(Int16, 
                            '/sensor/range/ultrasonic/right', 
                            lambda msg: rightUltrasonic_callback(node, msg), 
                            commum_qos_profile)
    
    node.create_subscription(Int16, 
                            '/sensor/range/ultrasonic/left', 
                            lambda msg: leftUltrasonic_callback(node, msg), 
                            commum_qos_profile)
    
    node.create_subscription(Int16, 
                            '/sensor/range/ultrasonic/back', 
                            lambda msg: backUltrasonic_callback(node, msg), 
                            commum_qos_profile)
    
    # ----------------- Velocity command (from the joystick or position controller)
    node.create_subscription(Twist, 
                            '/cmd_vel', 
                            lambda msg: cmdVel_callback(node, msg), 
                            commum_qos_profile)
    

    # ----------------- Joystick commands and connection status (from the firmware)
    node.create_subscription(Joy, 
                            '/joy/controller/ps4', 
                            lambda msg: abort_callback(node, msg), 
                            10)


    node.create_subscription(Bool,
                            '/joy/controller/connected',
                            lambda msg: joyConnected_callback(node, msg),
                            10)
    

    node.create_subscription(Int16, 
                            '/joy/controller/ps4/battery', 
                            lambda msg: joyBattery_callback(node, msg), 
                            10)


    # ---------------- Robot velocity from the odometry 
    node.create_subscription(Odometry, 
                            '/odom', 
                            lambda msg: odom_callback(node, msg), 
                            commum_qos_profile)


########################################################
# --------------- ODOMETRY
########################################################

def odometry_config(node: Node): 

    global commum_qos_profile, imu_qos_profile

    # --------------- Encoders sensors (from the firmware)
    node.create_subscription(Int32, 
                                '/esp_back/power/status/distance/ticks/left', 
                                lambda msg: ticksLeft_callback(node, msg),
                                10)

    node.create_subscription(Int32, 
                                '/esp_back/power/status/distance/ticks/left', 
                                lambda msg: ticksLeft_callback(node, msg),
                                10)
    
    # self.create_subscription(Int32, 
    #                             '/esp_front/power/status/distance/ticks/right', 
    #                             self.ticksRight_callback, 
    #                             10)

    # self.create_subscription(Int32, 
    #                             '/esp_front/power/status/distance/ticks/right', 
    #                             self.ticksRight_callback, 
    #                             10)
    
    # --------------- IMU sensors (from the firmware)
    node.create_subscription(Imu, 
                                '/sensor/orientation/imu', 
                                lambda msg: heading_callback(node, msg), 
                                imu_qos_profile)
    
    # --------------- Request to reset the odometry
    node.create_subscription(Bool, 
                                '/odom/reset', 
                                lambda msg: odomReset_callback(node, msg), 
                                commum_qos_profile)




########################################################
# --------------- LED MANAGER
########################################################

def led_config(node: Node): 

    global commum_qos_profile

    # ---------------- For debug, indicates when the robot stops by a collision alert 
    node.create_subscription(Bool,
                                '/safety/abort/collision_alert', 
                                lambda msg: collision_callback(node, msg),
                                commum_qos_profile)


    # ---------------- For debug, indicates when the robot stops by the user command 
    node.create_subscription(Bool, 
                                '/safety/abort/user_command',
                                lambda msg: manualAbort_callback(node, msg),
                                commum_qos_profile )
    
    # ---------------- For debug, indicates when the ultrasonics are disabled  
    node.create_subscription(Bool, 
                                '/safety/ultrasonic/disabled', 
                                lambda msg: ultrasonicStatus_callback(node, msg), 
                                commum_qos_profile )

    # ---------------- For debug, indicates when the joystick isn't connected 
    node.create_subscription(Bool, 
                                '/joy/controller/connected', 
                                lambda msg: joyConnected_callback(node, msg),
                                commum_qos_profile )

    # ---------------- Robot state 
    node.create_subscription(Int16,
                                '/machine_states/robot_state',
                                lambda msg: robotState_callback(node,msg), 
                                commum_qos_profile )
    
    # --------------- For debug indicates when the odometry is reseted 
    node.create_subscription(Bool, 
                                '/odom/reset', 
                                lambda msg: odomReset_callback(node, msg), 
                                commum_qos_profile)

    # --------------- Enable the LED when the robot reaches the goal 
    node.create_subscription(Bool, 
                                '/goal_manager/goal/sinalization', 
                                lambda msg: goalSinalization_callback(node, msg), 
                                commum_qos_profile)


    node.create_subscription(Int16, 
                            '/main_robot/autonomous_state',
                             lambda msg: autonomousMachine_callback(node, msg),
                              commum_qos_profile )

########################################################
# --------------- JOY ESP INTERFACE 
########################################################

def joy_config(node: Node): 

    global commum_qos_profile

    # -------- Get joytstick commands from firmware
    node.create_subscription(Joy, 
                                '/joy/controller/ps4', 
                                lambda msg: joy_callback(node, msg), 
                                10)


    # -------- Get robot states 
    node.create_subscription(Int16, 
                                '/machine_states/robot_state', 
                                lambda msg: robotState_callback(node, msg), 
                                commum_qos_profile)



########################################################
# ------------- Get callbacks 
########################################################


#  Joystick callbacks -----------------------------


def joyConnected_callback(node: Node, connection_status): 

    node.joy_connected = connection_status.data

    node.last_joy_connected = node.get_clock().now()


def joyBattery_callback(node: Node, battery): # get joytick battery 

    node.joy_battery = battery.data



#  Ultrasonics callbacks --------------------------


def rightUltrasonic_callback(node: Node, distance): 
    
    node.right_ultrasonic_distance = distance.data


def leftUltrasonic_callback(node: Node, distance): 
    
    node.left_ultrasonic_distance = distance.data


def backUltrasonic_callback(node: Node, distance): 
    
    node.back_ultrasonic_distance = distance.data


#  Velocity control -----------------------------

def odom_callback(node: Node, msg): # get current robot vel
    
    node.robot_vel.linear.x = msg.twist.twist.linear.x
    node.robot_vel.angular.z = msg.twist.twist.angular.z


 
def cmdVel_callback(node: Node, velocity): # get the vel command sent by others nodes
    
    node.cmd_vel.linear.x = velocity.linear.x
    node.cmd_vel.angular.z = velocity.angular.z

    node.last_vel_command_time = node.get_clock().now()
    


#  User abort command ------------------------------

def abort_callback(node: Node, user_command): # when received a emergency brake command, forces the robot to stop and keep it still until the button is released


    node.abort_flag = user_command.buttons[0] 
    SafeTwistNode.abort_command(node)


# Encoder ticks -----------------------------------

def ticksLeft_callback(node: Node, ticks_msg): 

    node.left_wheels_ticks = ticks_msg.data


def ticksRight_callback(node: Node, ticks_msg): 
    
    node.right_wheels_ticks = ticks_msg.data


# Imu callback ------------------------------------

def heading_callback(node: Node, imu_msg): 
    
    node.robot_quaternion = imu_msg

    # get the yaw angle
    node.robot_heading = tf3d.euler.quat2euler([node.robot_quaternion.orientation.w, 
                                                node.robot_quaternion.orientation.x, 
                                                node.robot_quaternion.orientation.y, 
                                                node.robot_quaternion.orientation.z])[2]
    
# Reset odometry command -------------------------

def odomReset_callback(node: Node, reset_msg):
    
    node.reset_odom = reset_msg.data



# Imminent collision detected by the ultrasonic sensors 

def collision_callback(node: Node, msg):

    node.collision_detected = msg.data



# Manual stop command send by the joystick 

def manualAbort_callback(node: Node, msg): 

    node.user_stop_command = msg.data



# Signal if the ultrasonics are disabled 
def ultrasonicStatus_callback(node: Node, msg): 

    if msg.data: 

        node.ultrasonic_disabled = True
    
    else: 

        node.ultrasonic_disabled = False



# Request from the goal manager to enable the led strip when the robot reaches a goal 
def goalSinalization_callback(node: Node, msg): 


    node.led_sinalization = msg.data

    LedManagerNode.goal_sinalization(node)
        



# Current robot state ---------------------------------------
def robotState_callback(node: Node, msg): 
    
    node.robot_state = msg.data

    node.manual_mode = (msg.data == node.ROBOT_MANUAL)



def joy_callback(node: Node, msg): 

    node.joy_msg = msg

    JoyInterfaceNode.joy_command(node)



def autonomousMachine_callback(node: Node, msg): 

    node.autonomous_state = msg.data