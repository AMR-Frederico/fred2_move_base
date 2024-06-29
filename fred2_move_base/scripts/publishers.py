import fred2_move_base.scripts.qos as qos 

from rclpy.node import Node

from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

commum_qos = qos.general_configuration()

def safe_config(node: Node):

    global commum_qos


    # ---------------- Safe velocity for the robot 
    node.safeVel_pub = node.create_publisher(Twist, 
                                            '/cmd_vel/safe', 
                                            commum_qos)

    # ---------------- For debug, indicates when the robot stops by the user command 
    node.userStop_pub = node.create_publisher(Bool, 
                                            '/safety/abort/user_command', 
                                            commum_qos)
    
    # ---------------- For debug, indicates when the robot stops by a collision alert 
    node.collisionDetection_pub = node.create_publisher(Bool, 
                                                '/safety/abort/collision_alert', 
                                                commum_qos)
    
    # ---------------- For debug, indicates when the ultrasonics are disabled  
    node.ultrasonicDisabled_pub = node.create_publisher(Bool, 
                                                        '/safety/ultrasonic/disabled', 
                                                        commum_qos)

    # ---------------- Indicates if the robot is in a safety zone 
    node.robotSafety_pub = node.create_publisher(Bool, 
                                                '/robot_safety', 
                                                commum_qos)
    
    # ---------------- Publish false if the joy connection is lost
    node.joyConnect_pub = node.create_publisher(Bool, 
                                                '/joy/controller/connected', 
                                                5)



########################################################
# --------------- ODOMETRY
########################################################


def odometry_config(node: Node):

    global commum_qos

    node.odom_pub = node.create_publisher(Odometry, '/odom', commum_qos)



########################################################
# --------------- LED MANAGER 
########################################################


def led_config(node: Node):

    global commum_qos

    # --------------- Main colors and sinalization, like the ones for machine states and goal sinalization 
    node.ledColor_pub = node.create_publisher(Int16, 
                                                '/cmd/led_strip/color', 
                                                 commum_qos)
    
    # --------------- Additional coloers for debug 
    node.ledDebug_pub = node.create_publisher(Int16, 
                                                '/cmd/led_strip/debug/color', 
                                                 commum_qos)


########################################################
# --------------- LED MANAGER 
########################################################


def joy_config(node: Node): 

    global commum_qos

    # --------- Velocity command 
    node.vel_pub = node.create_publisher(Twist, '/cmd_vel', commum_qos)      

    # --------- Request for reset the odometry 
    node.resetOdom_pub = node.create_publisher(Bool, '/odom/reset', commum_qos)

    # --------- Swicht between Manuel and Autonomous mode  
    node.switchMode_pub = node.create_publisher(Bool, '/joy/machine_states/switch_mode', commum_qos) 






def safe_publish(node: Node): 

    #################################    Messages       #########################################################################

    node.robotSafety_msg.data = node.robot_safety
    node.userStop_msg.data = node.user_abort_command
    node.collisionDetection_msg.data = node.stop_by_obstacle
    node.ultrasonicDisabled_msg.data = node.DISABLE_ULTRASONICS
                

    #################################    Publishers       #########################################################################

    node.safeVel_pub.publish(node.cmd_vel_safe)
    node.userStop_pub.publish(node.userStop_msg)
    node.robotSafety_pub.publish(node.robotSafety_msg)
    node.collisionDetection_pub.publish(node.collisionDetection_msg)
    node.ultrasonicDisabled_pub.publish(node.ultrasonicDisabled_msg)