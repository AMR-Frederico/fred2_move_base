import fred2_move_base.fred2_move_base.scripts.qos as qos

from rclpy.node import Node 

from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

########################################################
# --------------- Set subscribers 
########################################################

def safe_config(node: Node): 

    commum_qos_profile = qos.general_configuration


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


    if (node.abort_flag > node.abort_previous_flag): 

        node.user_abort_command = not node.user_abort_command


    node.abort_previous_flag = node.abort_flag
    
    # Stop Motor
    if node.user_abort_command: 
        
        node.get_logger().warn('User STOP command -> Stopping the robot')

        node.cmd_vel_safe.linear.x = node.robot_vel.linear.x * node.MOTOR_BRAKE_FACTOR
        node.cmd_vel_safe.angular.z = node.robot_vel.angular.z * node.MOTOR_BRAKE_FACTOR

