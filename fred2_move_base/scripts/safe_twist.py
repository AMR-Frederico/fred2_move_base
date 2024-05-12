#!/usr/bin/env python3

import rclpy
import threading
import sys

from typing import List, Optional

from rclpy.context import Context 
from rclpy.node import Node, ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult, FloatingPointRange
from rclpy.parameter import Parameter, ParameterType
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# Node execution arguments 
debug_mode = '--debug' in sys.argv

class SafeTwistNode(Node):


    # ---- ultrasonic readings 
    left_ultrasonic_distance = 1000
    right_ultrasonic_distance = 1000
    back_ultrasonic_distance = 1000

    # ---- detection variables 
    nearest_object_distance = 1000
    stop_by_obstacle = False 
    collisionDetection_msg = Bool()
    ultrasonicDisabled_msg = Bool()
    
    # ----- velocity control
    robot_vel = Twist()             # robot's velocity according to odometry           
    cmd_vel = Twist()               # velocity command from joystick (manual mode) or position controller (autonomous mode)
    cmd_vel_safe = Twist()          # safe velocity for the robot considerings the environment 

    # ---- joystick status
    joy_connected = False
    joy_battery = 0
    
    # ---- immediate stop request by user 
    user_abort_command = True       # robot starts in emergency state 
    abort_previous_flag = False
    userStop_msg = Bool()
    
    # ---- tells if the robot is safe, considering all the sensors and the environment
    robot_safety = False
    robotSafety_msg = Bool()

    # ---- deceleration factor when an obstacle is detected
    deceleration_factor = 1 


    def __init__(self, 
                node_name: str, 
                *, # keyword-only argument
                context: Context = None, 
                cli_args: List[str] = None, 
                namespace: str = None, 
                use_global_arguments: bool = True, 
                enable_rosout: bool = True, 
                start_parameter_services: bool = True, 
                parameter_overrides: List[Parameter] | None = None) -> None:
        
        super().__init__(node_name, 
                        context=context, 
                        cli_args=cli_args, 
                        namespace=namespace, 
                        use_global_arguments=use_global_arguments, 
                        enable_rosout=enable_rosout, 
                        start_parameter_services=start_parameter_services, 
                        parameter_overrides=parameter_overrides)
    

        self.quality_protocol() 
        self.setup_subscribers()
        self.setup_publishers()


        self.load_params()                                              # parameters initialization
        self.add_on_set_parameters_callback(self.parameters_callback)   # updates the parameters when they are updated by the command line
        
        
        # time threshold
        self.last_joy_connected = self.get_clock().now()
        self.last_vel_command_time = self.get_clock().now()



    def quality_protocol(self):

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Set the reliability policy to RELIABLE, ensuring reliable message delivery
            durability= QoSDurabilityPolicy.VOLATILE,   # Set the durability policy to VOLATILE, indicating messages are not stored persistently
            history=QoSHistoryPolicy.KEEP_LAST,         # Set the history policy to KEEP_LAST, storing a limited number of past messages
            depth=10,                                   # Set the depth of the history buffer to 10, specifying the number of stored past messages
            liveliness=QoSLivelinessPolicy.AUTOMATIC    # Set the liveliness policy to AUTOMATIC, allowing automatic management of liveliness
            
        )

    def setup_subscribers(self): 

        # --------------- Ultrasonic sensors (from the firmware)
        self.create_subscription(Int16, 
                                '/sensor/range/ultrasonic/right', 
                                self.rightUltrasonic_callback, 
                                self.qos_profile)
        

        self.create_subscription(Int16, 
                                '/sensor/range/ultrasonic/left', 
                                self.leftUltrasonic_callback, 
                                self.qos_profile)
        

        self.create_subscription(Int16, 
                                '/sensor/range/ultrasonic/back', 
                                self.backUltrasonic_callback, 
                                self.qos_profile)
        

        # ----------------- Velocity command (from the joystick or position controller)
        self.create_subscription(Twist, 
                                '/cmd_vel', 
                                self.cmdVel_callback, 
                                self.qos_profile)
        

        # ----------------- Joystick commands and connection status (from the firmware)
        self.create_subscription(Joy, 
                                '/joy/controller/ps4', 
                                self.abort_callback, 
                                10)


        self.create_subscription(Bool,
                                '/joy/controller/connected',
                                self.joyConnected_callback,
                                10)
        

        self.create_subscription(Int16, 
                                '/joy/controller/ps4/battery', 
                                self.joyBattery_callback, 
                                10)


        # ---------------- Robot velocity from the odometry 
        self.create_subscription(Odometry, 
                                '/odom', 
                                self.odom_callback, 
                                self.qos_profile)
            
        
    def setup_publishers(self):

        # ---------------- Safe velocity for the robot 
        self.safeVel_pub = self.create_publisher(Twist, 
                                                '/cmd_vel/safe', 
                                                self.qos_profile)

        # ---------------- For debug, indicates when the robot stops by the user command 
        self.userStop_pub = self.create_publisher(Bool, 
                                                '/safety/abort/user_command', 
                                                self.qos_profile)
        
        # ---------------- For debug, indicates when the robot stops by a collision alert 
        self.collisionDetection_pub = self.create_publisher(Bool, 
                                                    '/safety/abort/collision_alert', 
                                                    self.qos_profile)
        
        # ---------------- For debug, indicates when the ultrasonics are disabled  
        self.ultrasonicDisabled_pub = self.create_publisher(Bool, 
                                                            '/safety/ultrasonic/disabled', 
                                                            self.qos_profile)

        # ---------------- Indicates if the robot is in a safety zone 
        self.robotSafety_pub = self.create_publisher(Bool, 
                                                    '/robot_safety', 
                                                    self.qos_profile)
        
        # ---------------- Publish false if the joy connection is lost
        self.joyConnect_pub = self.create_publisher(Bool, 
                                                    '/joy/controller/connected', 
                                                    5)




    # updates the parameters when they are changed by the command line
    def parameters_callback(self, params):
        
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")


        if param.name == 'obstacle_detection_threshold':
            self.OBSTACLE_DETECTION_THRESHOLD = param.value
    

        if param.name == 'motor_brake_factor':
            self.MOTOR_BRAKE_FACTOR = param.value


        if param.name == 'max_linear_speed': 
            self.MAX_LINEAR_SPEED = param.value


        if param.name == 'max_angular_speed': 
            self.MAX_ANGULAR_SPEED = param.value

        
        if param.name == 'joy_low_battery_threshold': 
            self.JOY_LOW_BATTERY_THRESHOLD = param.value


        if param.name =='cmd_vel_timeout': 
            self.VEL_TIMEOUT = param.value


        if param.name == 'joy_timeout': 
            self.JOY_TIMEOUT = param.value 


        if param.name == 'disable_ultrasonics': 
            self.DISABLE_ULTRASONICS = param.value
        

        if param.name == 'frequency': 
            self.FREQUENCY = param.value 


        if param.name == 'debug': 
            self.DEBUG = param.value


        return SetParametersResult(successful=True)



    # initialize the node parameters 
    def load_params(self):

        # Declare parameters related to ultrasonic sensors, velocity limits, and debugging/testing
        self.declare_parameters(
            namespace='',
            parameters=[
                ('disable_ultrasonics', None, 
                    ParameterDescriptor(
                        description='Toggle ultrasonic sensors on/off', 
                        type=ParameterType.PARAMETER_BOOL)),

                ('obstacle_detection_threshold', None, 
                    ParameterDescriptor(
                        description='Stopping distance in cm when detecting an object with ultrasonic sensors', 
                        type=ParameterType.PARAMETER_DOUBLE)),

                ('max_angular_speed', None, 
                    ParameterDescriptor(
                        description='Maximum angular speed in degrees per second', 
                        type=ParameterType.PARAMETER_DOUBLE)),

                ('max_linear_speed', None, 
                    ParameterDescriptor(
                        description='Maximum linear speed in meters per second', 
                        type=ParameterType.PARAMETER_DOUBLE)),

                ('motor_brake_factor', None, 
                    ParameterDescriptor(
                        description='Brake factor for motors; negative values indicate braking force', 
                        type=ParameterType.PARAMETER_INTEGER)),
                
                ('joy_low_battery_threshold', None, 
                    ParameterDescriptor(
                        description='Define a threshold for low joystick battery', 
                        type=ParameterType.PARAMETER_INTEGER)),

                ('cmd_vel_timeout', None, 
                    ParameterDescriptor(
                        description='Timeout duration (in nanoseconds) to reset the cmd_vel status if no message is received within this time.', 
                        type=ParameterType.PARAMETER_INTEGER)),
                
                ('joy_connection_timeout', None, 
                    ParameterDescriptor(
                        description='Timeout duration (in nanoseconds) to reset the joy_connected status if no message is received within this time', 
                        type=ParameterType.PARAMETER_INTEGER)),

                ('frequency', None, 
                    ParameterDescriptor(
                        description='Node frequency', 
                        type=ParameterType.PARAMETER_INTEGER)),

                ('debug', None, 
                    ParameterDescriptor(
                        description='Enable debug prints for troubleshooting', 
                        type=ParameterType.PARAMETER_BOOL))
                
            ]
        )

        self.OBSTACLE_DETECTION_THRESHOLD = self.get_parameter('obstacle_detection_threshold').value 
        self.DISABLE_ULTRASONICS = self.get_parameter('disable_ultrasonics').value

        self.MOTOR_BRAKE_FACTOR = self.get_parameter('motor_brake_factor').value
        self.MAX_LINEAR_SPEED = self.get_parameter('max_linear_speed').value
        self.MAX_ANGULAR_SPEED = self.get_parameter('max_angular_speed').value
        
        self.JOY_LOW_BATTERY_THRESHOLD = self.get_parameter('joy_low_battery_threshold').value

        self.JOY_TIMEOUT = self.get_parameter('joy_connection_timeout').value
        self.VEL_TIMEOUT = self.get_parameter('cmd_vel_timeout').value

        self.DEBUG = self.get_parameter('debug').value
        self.FREQUENCY = self.get_parameter('frequency').value


    #################################    Joystick callbacks       #########################################################################

    
    def joyConnected_callback(self, connection_status): 

        self.joy_connected = connection_status.data

        self.last_joy_connected = self.get_clock().now()


    # get joytick battery 
    def joyBattery_callback(self, battery): 

        self.joy_battery = battery.data


    #################################    Ultrasonics callbacks       #########################################################################



    def rightUltrasonic_callback(self, distance): 
        
        self.right_ultrasonic_distance = distance.data
    


    def leftUltrasonic_callback(self, distance): 
        
        self.left_ultrasonic_distance = distance.data



    def backUltrasonic_callback(self, distance): 
        
        self.back_ultrasonic_distance = distance.data



    #################################    Velocity control       #########################################################################

    # get current robot vel 
    def odom_callback(self, msg): 
        
        self.robot_vel.linear.x = msg.twist.twist.linear.x
        self.robot_vel.angular.z = msg.twist.twist.angular.z
    


    # get the vel command sent by others nodes 
    def cmdVel_callback(self, velocity): 
        
        self.cmd_vel.linear.x = velocity.linear.x
        self.cmd_vel.angular.z = velocity.angular.z

        self.last_vel_command_time = self.get_clock().now()
    


    #################################    User abort command       #########################################################################

    # when received a emergency brake command, forces the robot to stop and keep it still until the button is released
    def abort_callback(self, user_command): 


        self.abort_flag = user_command.buttons[0] 


        if (self.abort_flag > self.abort_previous_flag): 

            self.user_abort_command = not self.user_abort_command



        self.abort_previous_flag = self.abort_flag
        
        # Stop Motor
        if self.user_abort_command: 
            
            self.get_logger().warn('User STOP command -> Stopping the robot')

            self.cmd_vel_safe.linear.x = self.robot_vel.linear.x * self.MOTOR_BRAKE_FACTOR
            self.cmd_vel_safe.angular.z = self.robot_vel.angular.z * self.MOTOR_BRAKE_FACTOR



    def ultrasonic_obstacle_check(self):
        

        # checks if any ultrasonics detected an obstacle closer than threshold
        if( self.right_ultrasonic_distance < self.OBSTACLE_DETECTION_THRESHOLD or
            self.left_ultrasonic_distance  < self.OBSTACLE_DETECTION_THRESHOLD or
            self.back_ultrasonic_distance  < self.OBSTACLE_DETECTION_THRESHOLD ): 

                self.stop_by_obstacle = True

                self.get_logger().warn('Obstacle detect! Stopping the robot!')


        # Slows down the robot depending on the distance from the obstacle
        else: 
            
            self.stop_by_obstacle = False

            distances = [self.right_ultrasonic_distance, self.left_ultrasonic_distance, self.back_ultrasonic_distance]
            self.nearest_object_distance = min(distances)


            # applies a braking factor based on the distance detected by the sensors
            self.deceleration_factor = self.nearest_object_distance/(2 * self.OBSTACLE_DETECTION_THRESHOLD)

        
            if self.deceleration_factor > 1: 

                self.deceleration_factor = 1
        

            if self.deceleration_factor < 0.25: 

                self.deceleration_factor = 0





    def safe_twist(self): 
        
        # Ensure that user_abort_command is not triggered
        if not self.user_abort_command: 
            
            
            current_time = self.get_clock().now()

            # Reset joy_connected status if no message received within the joystick timeout
            if (current_time - self.last_joy_connected).nanoseconds > 1e5 and self.JOY_TIMEOUT: 

                self.joy_connected = False
                self.get_logger().warn('Joy connection set to FALSE due to a timeout (no message received within the last 1 and half second).')

                joy_status_msg = Bool()
                joy_status_msg.data = False
                self.joyConnect_pub.publish(joy_status_msg)

            
            # Reset joy_connected status if no message received within the vel timeout
            if (current_time - self.last_vel_command_time).nanoseconds > self.VEL_TIMEOUT: 
                
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel.linear.x = 0.0

                self.get_logger().warn('Reset vel due to a timeout (no message received within the last 1 and half second).')
            

            # If ultrasonics are disabled, reset obstacle detection settings
            if self.DISABLE_ULTRASONICS: 
                
                self.stop_by_obstacle = False
                self.deceleration_factor = 1
            
                # self.get_logger().warn('The ultrasonic were deactivated')
    

            else: 
                # Check for obstacles using ultrasonic sensors
                self.ultrasonic_obstacle_check()

            
            # Adjust velocity based on obstacle detection and joy connection status
            if self.stop_by_obstacle or not self.joy_connected: 
                
                # Apply braking factor, to immediately stops the robot, if obstacle detected or joy not connected
                self.cmd_vel_safe.linear.x = self.linear.x * self.MOTOR_BRAKE_FACTOR
                self.cmd_vel_safe.angular.z = self.robot_vel.angular.z * self.MOTOR_BRAKE_FACTOR



            else: 
                # Apply deceleration factor otherwise
                self.cmd_vel_safe.linear.x = self.cmd_vel.linear.x * self.DECELERATION_FACTOR
                self.cmd_vel_safe.angular.z = self.cmd_vel.angular.z * self.DECELERATION_FACTOR



            # Velocity saturation
            self.cmd_vel_safe.linear.x = max(min(self.cmd_vel_safe.linear.x, self.MAX_LINEAR_SPEED), - self.MAX_LINEAR_SPEED)
            self.cmd_vel_safe.angular.z = max(min(self.cmd_vel_safe.angular.z, self.MAX_ANGULAR_SPEED), - self.MAX_ANGULAR_SPEED)    


        # Check for conditions indicating unsafe robot operation
        if self.stop_by_obstacle or self.user_abort_command or not self.joy_connected:

            self.robot_safety = False
            self.get_logger().warn(f'Robot NOT safe -> obstacle: {self.stop_by_obstacle} | stop command: {self.user_abort_command} | joy connected: {self.joy_connected}')
        
        else: 
            
            self.robot_safety = True # Indicate safe operation if no unsafe conditions detected


        # Log warning if joystick battery is low
        if self.joy_battery < self.JOY_LOW_BATTERY_THRESHOLD: 

            self.get_logger().warn(f'!!!!!!!!!!!!!! JOYSTICK WITH LOW BATTERY: {self.joy_battery} !!!!!!!!!!!!!!!!!!!!!!!')

        else: 

            self.get_logger().warn(f'Joy battery: {self.joy_battery}%')


        #################################    Messages       #########################################################################

        self.robotSafety_msg.data = self.robot_safety
        self.userStop_msg.data = self.user_abort_command
        self.collisionDetection_msg.data = self.stop_by_obstacle
        self.ultrasonicDisabled_msg.data = self.DISABLE_ULTRASONICS
                    

        #################################    Publishers       #########################################################################

        self.safeVel_pub.publish(self.cmd_vel_safe)
        self.userStop_pub.publish(self.userStop_msg)
        self.robotSafety_pub.publish(self.robotSafety_msg)
        self.collisionDetection_pub.publish(self.collisionDetection_msg)
        self.ultrasonicDisabled_pub.publish(self.ultrasonicDisabled_msg)
            
        

        if debug_mode or self.DEBUG: 

            self.get_logger().warn(f'Robot safety -> {self.robot_safety}')
            self.get_logger().info(f'Emergency mode -> user comand abort: {self.user_abort_command} | collision detected: {self.stop_by_obstacle}')
            self.get_logger().info(f'Emergency mode -> joy connected: {self.joy_connected} | ultrasonics disabled: {self.DISABLE_ULTRASONICS}')
            self.get_logger().info(f'Ultrasonics -> left: {self.left_ultrasonic_distance} | right: {self.right_ultrasonic_distance} | Back: {self.back_ultrasonic_distance}')
            self.get_logger().info(f'Robot velocity -> linear: {self.robot_vel.linear.x} | angular: {self.robot_vel.angular.z}')
            self.get_logger().info(f'Velocity command -> linear: {self.cmd_vel.linear.x} | angular: {self.cmd_vel.angular.z} | braking_factor: {self.deceleration_factor}')
            self.get_logger().info(f'Safe velocity command -> linear: {self.cmd_vel_safe.linear.x} | angular: {self.cmd_vel_safe.angular.z}\n')



if __name__ == '__main__':
    # Create a custom context for single thread and real-time execution
    rclpy.init()

    safe_context = rclpy.Context()
    safe_context.init()
    safe_context.use_real_time = True
    

    node = SafeTwistNode(
        node_name='safe_twist',
        context=safe_context,
        cli_args=['--debug'],
        namespace='move_base',
        enable_rosout=False
    )

    # Make the execution in real time 
    executor = SingleThreadedExecutor(context=safe_context)
    executor.add_node(node)

    # create a separate thread for the callbacks and another for the main function 
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(node.FREQUENCY)

    try: 
        while rclpy.ok(): 
            node.safe_twist()
            rate.sleep()
        
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
