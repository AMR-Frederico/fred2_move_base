#!/usr/bin/env python3

import rclpy
import threading
import sys

import fred2_move_base.fred2_move_base.scripts.publishers as publishers
import fred2_move_base.fred2_move_base.scripts.subscribers as subscribers
import fred2_move_base.fred2_move_base.scripts.parameters as params 

from typing import List, Optional

from rclpy.context import Context 
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


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
            
        
        publishers.safe_publish(self)


        if debug_mode or self.DEBUG: 


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
