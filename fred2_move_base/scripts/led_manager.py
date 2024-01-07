#!/usr/bin/env python3

import rclpy
import yaml
import os
import sys
import threading

from typing import Any, List, Optional

from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.time import Duration, Time

from rcl_interfaces.msg import Parameter
from std_msgs.msg import Bool, Int16, Float32
from geometry_msgs.msg import PoseStamped, Pose2D

# Parameters file (yaml)
led_path = '~/ros2_ws/src/fred2_move_base/config/move_base_params.yaml'
led_group = 'led_manager'

led_color = Int16()  # Publisher message

# Check for cli_args 
debug_mode = '--debug' in sys.argv

class led_manager(Node): 

    
    robot_state = -5    # Robot state, random initial value

    last_goal_reached = False

    last_stop_command = False
    is_emergency_stop = True

    collision_detected = False

    ultrasonic_disabled = False

    goal_pose = Pose2D()

    led_goal_reached = False
    led_goal_signal = False

    LED_ON_TIME = Duration(seconds=1) # It's also possible to specify nanoseconds

    
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

        # Load parameters from YAML file
        self.load_params(led_path, led_group)
        self.get_colors()
        
        self.start_time = self.get_clock().now()


        self.collisionDetection_sub = self.create_subscription(
                                    Bool,
                                    '/safety/abort/colision_detection', 
                                    self.collision_callback,
                                    10 )

        self.manualAbort_sub = self.create_subscription(
                                    Bool, 
                                    '/safety/abort/user_command',
                                    self.manual_abort_callback,
                                    10 )
        
        self.ultrasonicStatus_sub = self.create_subscription(Bool, 
                                                             '/safety/ultrasonic/disabled', 
                                                             self.ultrasonicStatus_callback, 
                                                             10 )
        
        self.robotState_sub = self.create_subscription(
                                    Int16,
                                    '/machine_states/main/robot_status',
                                    self.robot_state_callback, 
                                    10 )

        self.currentGoal_sub = self.create_subscription(
                                    PoseStamped,
                                    '/goal_manager/goal/current',
                                    self.goal_current_callback, 
                                    10 )
        
        self.goalReached_sub = self.create_subscription(
                                    Bool, 
                                    '/goal_manager/goal/reached', 
                                    self.goal_reached_callback, 
                                    10 )
        
        
        self.led_color_pub = self.create_publisher(Int16, 
                                                    '/cmd/led_strip/color', 
                                                    10 )
        

    def get_colors(self):
        # Get index colors
        self.WHITE = self.get_parameter('white').value
        self.BLUE = self.get_parameter('blue').value
        self.YELLOW = self.get_parameter('yellow').value
        self.PINK = self.get_parameter('pink').value
        self.ORANGE = self.get_parameter('orange').value 
        self.RED = self.get_parameter('red').value
        self.GREEN = self.get_parameter('green').value 
        self.BLACK = self.get_parameter('black').value 
        
        self.WAYPOINT_GOAL = self.get_parameter('waypoint_goal').value
        self.GHOST_GOAL = self.get_parameter('ghost_goal').value




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



    # Imminent collision detected by the ultrasonic sensors 
    def collision_callback(self, msg):

        self.collision_detected = msg.data



    # Manual stop command send by the joystick 
    def manual_abort_callback(self, msg): 

        user_stop_command = msg.data

        if (user_stop_command == True) and (self.last_stop_command == False): 
            self.is_emergency_stop = not self.is_emergency_stop

        self.last_stop_command = user_stop_command



    def ultrasonicStatus_callback(self, msg): 
        
        self.ultrasonic_disabled = msg.data



    def robot_state_callback(self, msg): 
        
        self.robot_state = msg.data



    def goal_current_callback(self, msg):
        
        self.goal_pose.x = msg.pose.position.x
        self.goal_pose.y = msg.pose.position.y
        self.goal_pose.theta = msg.pose.orientation.z



    def goal_reached_callback(self, msg):

        self.goal_reached = msg.data

        if (self.goal_reached == True) and (self.last_goal_reached == False): 
            
            self.start_time = self.get_clock().now()
            self.led_goal_reached = True
        
        else: 

            self.led_goal_reached = False

        self.last_goal_reached = self.goal_reached


        if (self.get_clock().now() - self.start_time) > self.LED_ON_TIME: 
            
            self.led_goal_reached = False 

        

        if self.goal_pose.theta == self.WAYPOINT_GOAL: 
        
            self.led_goal_signal = True
        

        if self.goal_pose.theta == self.GHOST_GOAL: 
            
            self.led_goal_signal = False
        
        
 
def main():


    if node.is_emergency_stop: 

        led_color.data = node.RED
    
    else: 

        if node.robot_state == 50: 

            led_color.data = node.BLUE


            if (node.led_goal_reached == True) and (node.led_goal_signal == True):

                led_color.data = node.GREEN


            if node.collision_detected == True: 

                led_color.data = node.ORANGE


            if node.ultrasonic_disabled == True: 

                led_color.data = node.YELLOW
    
    
    node.led_color_pub.publish(led_color) 


    if debug_mode: 

        node.get_logger().info(f"Color: {led_color}")
        node.get_logger().info(f"Collision alert: {node.collision_detected}")
        node.get_logger().info(f"Stop command: {node.is_emergency_stop}")
        node.get_logger().info(f"Robot sate: {node.robot_state}")
        node.get_logger().info(f"Goal reached: {node.led_goal_reached}")
        node.get_logger().info(f"Waypoint goal: {node.led_goal_signal}\n")


if __name__ == '__main__':
    
    rclpy.init()

    node = led_manager(
        'led_manager', 
        namespace='move_base',
        cli_args=['--debug'],
        enable_rosout=False, 
    )

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(10)

    try: 
        while rclpy.ok(): 
            
            main()
            rate.sleep()
        
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
