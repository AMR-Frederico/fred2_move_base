#!/usr/bin/env python3
import rclpy
import threading
import sys
import os 
import yaml

from typing import List, Optional

from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.context import Context 
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Bool, Float32, Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# Node execution arguments 
debug_mode = '--debug' in sys.argv

# Parameters file (yaml)
node_path = '~/ros2_ws/src/fred2_move_base/config/move_base_params.yaml'
node_group = 'safe_twist'


class SafeTwistNode(Node):

    smallest_reading = 1000

    stop_by_obstacle = False 

    left_ultrasonic_distance = 1000
    right_ultrasonic_distance = 1000
    back_ultrasonic_distance = 1000

    user_abort_command = True
    abort_previous_flag = False
    
    robot_user_abort = False

    robot_vel = Twist()

    cmd_vel_safe = Twist()
    cmd_vel = Twist()
    vel_timeout = False

    joy_connected = False
    
    robot_safety = False
    robotSafety_msg = Bool()

    userStop_msg = Bool()
    collisionDetection_msg = Bool()
    ultrasonicDisabled_msg = Bool()

    braking_factor = 1



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
        
        # quality protocol -> the node must not lose any message 
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=1
        )

        self.create_subscription(Float32, 
                                 '/sensor/range/ultrasonic/right', 
                                 self.rightUltrasonic_callback, 
                                 qos_profile)
        

        self.create_subscription(Float32, 
                                 '/sensor/range/ultrasonic/left', 
                                 self.leftUltrasonic_callback, 
                                 qos_profile)
        

        self.create_subscription(Float32, 
                                 '/sensor/range/ultrasonic/back', 
                                 self.backUltrasonic_callback, 
                                 qos_profile)
        

        self.create_subscription(Odometry, 
                                 '/odom', 
                                 self.odom_callback, 
                                 qos_profile)
        

        self.create_subscription(Twist, 
                                 '/cmd_vel', 
                                 self.cmdVel_callback, 
                                 qos_profile)
        

        self.create_subscription(Int16, 
                                 '/joy/controller/ps4/brake', 
                                 self.abort_callback, 
                                 qos_profile)


        self.create_subscription(Bool,
                                 '/joy/controller/connected',
                                 self.joyConnected_callback,
                                 1)

        
        self.safeVel_pub = self.create_publisher(Twist, 
                                                '/cmd_vel/safe', 
                                                qos_profile)


        self.userStop_pub = self.create_publisher(Bool, 
                                                  '/safety/abort/user_command', 
                                                  qos_profile)
        

        self.collisionDetection_pub = self.create_publisher(Bool, 
                                                      '/safety/abort/collision_alert', 
                                                      qos_profile)
        

        self.ultrasonicDisabled_pub = self.create_publisher(Bool, 
                                                            '/safety/ultrasonic/disabled', 
                                                            qos_profile)


        self.robotSafety_pub = self.create_publisher(Bool, 
                                                     '/robot_safety', 
                                                     qos_profile)
        

        self.joyConnect_pub = self.create_publisher(Bool, 
                                                    '/joy/controller/connected', 
                                                    1)


        # get params from the config file
        self.load_params(node_path, node_group)
        self.get_params()


        self.add_on_set_parameters_callback(self.parameters_callback)

        self.last_joy_connected = self.get_clock().now()
        self.last_vel_command_time = self.get_clock().now()



    def parameters_callback(self, params):
        
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")



        if param.name == 'safe_distance':
            self.SAFE_DISTANCE = param.value
    
  
        if param.name == 'motor_brake_factor':
            self.MOTOR_BRAKE_FACTOR = param.value


        if param.name == 'max_linear_speed': 
            self.MAX_LINEAR_SPEED = param.value


        if param.name == 'max_angular_speed': 
            self.MAX_ANGULAR_SPEED = param.value

        
        if param.name =='disable_ultrasonics': 
            self.DISABLE_ULTRASONICS = param.value



        return SetParametersResult(successful=True)



    def load_params(self, path, group): 
        param_path = os.path.expanduser(path)

        with open(param_path, 'r') as params_list: 
            params = yaml.safe_load(params_list)
        
        # Get the params inside the specified group
        params = params.get(group, {})

        # Declare parameters with values from the YAML file
        for param_name, param_value in params.items():
            # Adjust parameter name to lowercase
            param_name_lower = param_name.lower()
            self.declare_parameter(param_name_lower, param_value)
            self.get_logger().info(f'{param_name_lower}: {param_value}')



    def get_params(self): 
        self.SAFE_DISTANCE = self.get_parameter('safe_distance').value 
        self.MOTOR_BRAKE_FACTOR = self.get_parameter('motor_brake_factor').value
        self.MAX_LINEAR_SPEED = self.get_parameter('max_linear_speed').value
        self.MAX_ANGULAR_SPEED = self.get_parameter('max_angular_speed').value
        self.DISABLE_ULTRASONICS = self.get_parameter('disable_ultrasonics').value



    def joyConnected_callback(self, connection_status): 

        self.joy_connected = connection_status.data

        self.last_joy_connected = self.get_clock().now()



    def rightUltrasonic_callback(self, distance): 
        
        self.right_ultrasonic_distance = distance.data
    



    def leftUltrasonic_callback(self, distance): 
        
        self.left_ultrasonic_distance = distance.data



    def backUltrasonic_callback(self, distance): 
        
        self.back_ultrasonic_distance = distance.data




    # get current robot vel 
    def odom_callback(self, msg): 
        
        self.robot_vel.linear.x = msg.twist.twist.linear.x
        self.robot_vel.angular.z = msg.twist.twist.angular.z
    


    # get the vel command sent by others nodes 
    def cmdVel_callback(self, velocity): 
        
        self.cmd_vel.linear.x = velocity.linear.x
        self.cmd_vel.angular.z = velocity.angular.z

        self.last_vel_command_time = self.get_clock().now()
    


    # when received a emergency brake command, forces the robot to stop and keep it still until the button is released
    def abort_callback(self, user_command): 

        self.abort_flag = user_command.data 

        if (self.abort_flag > self.abort_previous_flag): 

            self.user_abort_command = not self.user_abort_command


        self.abort_previous_flag = self.abort_flag
        

        if self.user_abort_command: 

            self.cmd_vel_safe.linear.x = self.robot_vel.linear.x * self.MOTOR_BRAKE_FACTOR
            self.cmd_vel_safe.angular.z = self.robot_vel.angular.z * self.MOTOR_BRAKE_FACTOR



    def ultrasonic_obstacle_check(self):
        

        if( self.right_ultrasonic_distance < self.SAFE_DISTANCE or
            self.left_ultrasonic_distance  < self.SAFE_DISTANCE or
            self.back_ultrasonic_distance  < self.SAFE_DISTANCE ): 

                self.stop_by_obstacle = True

                self.get_logger().info('Obstacle detect! Stopping the robot!')


        # Slows down the robot depending on the distance from the obstacle
        else: 
            
            self.stop_by_obstacle = False

            distances = [self.right_ultrasonic_distance, self.left_ultrasonic_distance, self.back_ultrasonic_distance]
            smallest_reading = min(distances)


            # applies a braking factor based on the distance detected by the sensors
            self.braking_factor = smallest_reading/(2 * self.SAFE_DISTANCE)

        
            if self.braking_factor > 1: 

                self.braking_factor = 1
        

            if self.braking_factor < 0.25: 

                self.braking_factor = 0





    def safe_twist(self): 
        
        if self.DISABLE_ULTRASONICS: 
            
            self.stop_by_obstacle = False
            braking_factor = 1
        
            self.get_logger().warn('The ultrasonic were deactivated')
    

        else: 
            
            if not self.user_abort_command: 
                
                
                current_time = self.get_clock().now()

                # Timeout to reset the joy_connected status 
                if (current_time - self.last_joy_connected).nanoseconds > 2e9 and self.joy_connected: 
                    
                    self.joy_connected = False
                    self.get_logger().warn('Joy connection set to FALSE due to a timeout (no message received within the last 2 seconds).')

                    joy_status_msg = Bool()
                    joy_status_msg.data = False
                    self.joyConnect_pub.publish(joy_status_msg)

                

                # Timeout to reset the velocity command
                if (current_time - self.last_vel_command_time).nanoseconds > 2e9: 
                    
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_vel.linear.x = 0.0

                    self.get_logger().warn('Reset vel due to a timeout (no message received within the last 2 seconds).')
                
                
                self.ultrasonic_obstacle_check()

                
                if self.stop_by_obstacle or not self.joy_connected: 
                    
                    self.cmd_vel_safe.linear.x = self.robot_vel.linear.x * self.MOTOR_BRAKE_FACTOR
                    self.cmd_vel_safe.angular.z = self.robot_vel.angular.z * self.MOTOR_BRAKE_FACTOR



                else: 
                    
                    self.cmd_vel_safe.linear.x = self.cmd_vel.linear.x * self.braking_factor
                    self.cmd_vel_safe.angular.z = self.cmd_vel.angular.z * self.braking_factor


                # vel saturation
                self.cmd_vel_safe.linear.x = max(min(self.cmd_vel_safe.linear.x, self.MAX_LINEAR_SPEED), - self.MAX_LINEAR_SPEED)
                self.cmd_vel_safe.angular.z = max(min(self.cmd_vel_safe.angular.z, self.MAX_ANGULAR_SPEED), - self.MAX_ANGULAR_SPEED)
                
        

        
        if self.stop_by_obstacle or self.user_abort_command or not self.joy_connected:

            self.robot_safety = False
        
        else: 
            
            self.robot_safety = True




        self.robotSafety_msg.data = self.robot_safety
        self.userStop_msg.data = self.user_abort_command
        self.collisionDetection_msg.data = self.stop_by_obstacle
        self.ultrasonicDisabled_msg.data = self.DISABLE_ULTRASONICS
                    

        self.safeVel_pub.publish(self.cmd_vel_safe)
        self.userStop_pub.publish(self.userStop_msg)
        self.robotSafety_pub.publish(self.robotSafety_msg)
        self.collisionDetection_pub.publish(self.collisionDetection_msg)
        self.ultrasonicDisabled_pub.publish(self.ultrasonicDisabled_msg)
            
        

        if debug_mode: 

            self.get_logger().warn(f'Robot safety -> {self.robot_safety}')
            self.get_logger().info(f'Emergency mode -> user comand abort: {self.user_abort_command} | collision detected: {self.stop_by_obstacle}')
            self.get_logger().info(f'Emergency mode -> joy connected: {self.joy_connected} | ultrasonics disabled: {self.DISABLE_ULTRASONICS}')
            self.get_logger().info(f'Ultrasonics -> left: {self.left_ultrasonic_distance} | right: {self.right_ultrasonic_distance} | Back: {self.back_ultrasonic_distance}')
            self.get_logger().info(f'Robot velocity -> linear: {self.robot_vel.linear.x} | angular: {self.robot_vel.angular.z}')
            self.get_logger().info(f'Velocity command -> linear: {self.cmd_vel.linear.x} | angular: {self.cmd_vel.angular.z} | braking_factor: {self.braking_factor}')
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

    rate = node.create_rate(1)

    try: 
        while rclpy.ok(): 
            node.safe_twist()
            rate.sleep()
        
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
