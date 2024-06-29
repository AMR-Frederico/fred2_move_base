#!/usr/bin/env python3

import rclpy
import sys
import threading

import fred2_move_base.scripts.debug as debug
import fred2_move_base.scripts.subscribers as subscribers 
import fred2_move_base.scripts.publishers as publishers
import fred2_move_base.scripts.parameters as params  

from typing import Any, List, Optional

from rclpy.time import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter

from rcl_interfaces.msg import Parameter

from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D


# Check for cli_args 
debug_mode = '--debug' in sys.argv


class LedManagerNode(Node): 

    led_color = Int16()
    led_debug = Int16()
    

    robot_state = -5                # Current state of the robot, initialized to a random value


    user_stop_command = True        # Flag indicating whether a stop command is issued by the user
    collision_detected = False      # Flag indicating whether a collision is detected
    joy_connected = False           # Flag indicating whether the joystick is connected


    ultrasonic_disabled = False     # Flag indicating whether the ultrasonic sensors are disabled


    goal_pose = Pose2D()            # Current goal position and orientation for navigation

    goal_pose.x = 0.0 
    goal_pose.y = 0.0
    goal_pose.theta = 0.0


    odom_reset = False              # Flag indicating whether the odometry needs to be reset


    LED_ON_TIME = Duration(seconds=0.5)  # Duration for which the LED signal is kept on (in seconds)



    # starts with randon value 
    ROBOT_MANUAL = 1000
    ROBOT_AUTONOMOUS = 1000
    ROBOT_IN_GOAL = 1000
    ROBOT_MISSION_COMPLETED = 1000
    ROBOT_EMERGENCY = 1000

    
    def __init__(self, 
                 node_name: str, 
                 *, # keyword-only argument
                 cli_args: List[str] = None, 
                 namespace: str = None, 
                 use_global_arguments: bool = True, 
                 enable_rosout: bool = True, 
                 start_parameter_services: bool = True, 
                 parameter_overrides: List[Parameter] | None = None) -> None:
        
        super().__init__(node_name, 
                         cli_args=cli_args, 
                         namespace=namespace, 
                         use_global_arguments=use_global_arguments, 
                         enable_rosout=enable_rosout, 
                         start_parameter_services=start_parameter_services, 
                         parameter_overrides=parameter_overrides)

        subscribers.led_config(self)
        publishers.led_config(self)
        params.led_config(self)

        self.add_on_set_parameters_callback(params.led_params_callback) # updates the parameters when they are updated by the command line

        self.start_time = self.get_clock().now()

   
    def goal_sinalization(self): 

        if self.led_sinalization: 
                        
            self.led_goal_reached = True

            self.get_logger().warn('ROBOT IN GOAL')
            
            self.start_time = self.get_clock().now()


        # For keeping the signal on for a determined time
        if (self.get_clock().now() - self.start_time) > self.LED_ON_TIME: 
            
            self.led_goal_reached = False 


        
    def led_manager(self):

        # #* Colors for the robot state: 

        if self.robot_state == self.ROBOT_EMERGENCY: 

            self.led_color.data = self.RED

        
        else: 


            if self.robot_state == self.ROBOT_AUTONOMOUS: 

                self.led_color.data = self.BLUE


                if (self.led_goal_reached == True):

                    self.get_logger().warn('GOAL SIGNAL -> LED ON')
                    self.led_color.data = self.GREEN



            if self.robot_state == self.ROBOT_MISSION_COMPLETED:

                self.led_color.data = self.YELLOW




            if self.robot_state == self.ROBOT_MANUAL: 

                self.led_color.data = self.WHITE

        
        # #* Colors for debug the robot states:
                
        if self.collision_detected: 

            self.led_debug.data = self.ORANGE
        

        elif self.user_stop_command: 
            
            self.led_debug.data = self.PINK
        

        elif not self.joy_connected: 

            self.led_debug.data = self.PURPLE


        elif self.ultrasonic_disabled: 

            self.led_debug.data = self.YELLOW
        
        
        elif self.odom_reset: 
            
            self.led_debug.data = self.LIGHT_GREEN


        else: 
            
            self.led_debug.data = self.BLACK


        self.ledColor_pub.publish(self.led_color) 
        self.ledDebug_pub.publish(self.led_debug)


        if debug_mode or self.DEBUG: 

            self.get_logger().info(f"Color: {self.led_color.data}")
            self.get_logger().info(f"Debug color: {self.led_debug.data}")
            self.get_logger().info(f"Collision alert: {self.collision_detected}")
            self.get_logger().info(f"Stop command: {self.user_stop_command}")
            self.get_logger().info(f"Joy connected: {self.joy_connected}")
            self.get_logger().info(f"Ultrasonics disabled: {self.ultrasonic_disabled}")
            self.get_logger().info(f"Robot state: {self.robot_state}\n")


if __name__ == '__main__':
    
    rclpy.init()

    node = LedManagerNode(
        'led_manager', 
        namespace='move_base',
        cli_args=['--debug'],
        enable_rosout=False, 
        use_global_arguments=True, 
    )

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(node.FREQUENCY)

    try: 
        while rclpy.ok(): 
            
            node.led_manager()
            rate.sleep()
        
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
