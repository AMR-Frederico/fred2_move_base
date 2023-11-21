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

# Imminent collision detected by the ultrasonic sensors 
collision_detected = False

# Manual stop command send by the joystick 
user_stop_command = True
is_emergency_stop = False
last_stop_command = False

# Robot state, random initial value
robot_state = -5 

goal_pose = Pose2D()

WAYPOINT_GOAL = 0  # Goals points that the robot must signal
GHOST_GOAL = 1

LED_goal_signal = False

LED_ON_TIME = Duration(seconds=1) # It's also possible to specify nanoseconds
LED_goal_reached = False   

goal_reached = False
last_goal_reached = False       # Sinalization

led_color = Int16()  # Publisher message

# Check for cli_args 
debug_mode = '--debug' in sys.argv

class led_manager(Node): 
    
    white = 0
    blue = 0
    yellow = 0
    pink = 0 
    green = 0 
    orange = 0 
    red = 0 
    black = 0 
    
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

        self.collision_detection_sub = self.create_subscription(
                                    Bool,
                                    'safety/abort/colision_detection', 
                                    self.collision_callback,
                                    10 )

        self.manual_abort_sub = self.create_subscription(
                                    Bool, 
                                    'safety/abort/user_command',
                                    self.manual_abort_callback,
                                    10 )
        
        self.robot_state_sub = self.create_subscription(
                                    Int16,
                                    'machine_states/main/robot_status',
                                    self.robot_state_callback, 
                                    10 )

        self.current_goal_sub = self.create_subscription(
                                    PoseStamped,
                                    'goal_manager/goal/current',
                                    self.current_goal_callback, 
                                    10 )
        
        self.goal_reached_sub = self.create_subscription(
                                    Bool, 
                                    'goal_manager/goal/reached', 
                                    self.goal_reached_callback, 
                                    10 )
        
        self.led_color_pub = self.create_publisher(Int16, 
                                              '/cmd/led_strip/color', 
                                              10 )
        

    def get_colors(self):
        # Get index colors
        self.white = self.get_parameter('led_white').value
        self.blue = self.get_parameter('led_blue').value
        self.yellow = self.get_parameter('led_yellow').value
        self.pink = self.get_parameter('led_pink').value
        self.orange = self.get_parameter('led_orange').value 
        self.red = self.get_parameter('led_red').value 
        self.black = self.get_parameter('led_black').value 



    def load_params(self, path, group): 
        param_path = os.path.expanduser(path)

        with open(param_path, 'r') as params_list: 
            params = yaml.safe_load(params_list)
        
        # Get the params inside led_manager group
        params = params.get(group, {})

        # Declare parameters with values from the YAML file
        for param_name, param_value in params.items():
            self.declare_parameter(param_name, param_value)
            self.get_logger().info(f'{param_name}: {param_value}')
    


    def collision_callback(self, msg): 
        global collision_detected
        collision_detected = msg.data


    
    def manual_abort_callback(self, msg): 
        global user_stop_command
        global is_emergency_stopemergency_stop
        global last_stop_command

        user_stop_command = msg.data

        if (user_stop_command == True) and (last_stop_command == False): 
            is_emergency_stop = not is_emergency_stop

        last_stop_command = user_stop_command



    def robot_state_callback(self, msg): 
        global robot_state
        robot_state = msg.data



    def current_goal_callback(self, msg):
        global goal_pose
        goal_pose.x = msg.pose.position.x
        goal_pose.y = msg.pose.position.y
        goal_pose.theta = msg.pose.orientation.z



    def goal_reached_callback(self, msg):
        global LED_ON_TIME, LED_goal_reached
        global goal_reached, last_goal_reached
        global goal_pose
        global WAYPOINT_GOAL, GHOST_GOAL, LED_goal_signal

        goal_reached = msg.data

        if (goal_reached == True) and (last_goal_reached == False): 
            start_time = self.get_clock().now()
            LED_goal_reached = True
        
        last_goal_reached = goal_reached

        if (self.get_clock().now() - start_time) > LED_ON_TIME: 
            LED_goal_reached = False 

        if goal_pose.theta == WAYPOINT_GOAL: 
            LED_goal_signal = True
        
        if goal_pose.theta == GHOST_GOAL: 
            LED_goal_signal = False

 
def main():
    
    led_color.data = node.pink

    if robot_state == 50: 
        led_color.data = node.blue

        if (LED_goal_reached == True) and (LED_goal_signal == True):
            led_color.data = node.green

        if (collision_detected == True): 
            led_color.data = node.orange
    
    if robot_state == 2: 
        led_color.data = node.red
    
    node.led_color_pub.publish(led_color) 

    if debug_mode: 
        node.get_logger().info(f"\nLED_MANAGER:"
                               f"Color: {led_color} \n"
                               f"Collision alert: {collision_detected} \n"
                               f"Stop command: {user_stop_command} \n"
                               f"Robo tsate: {robot_state} \n"
                               f"Goal reached: {LED_goal_reached} \n"
                               f"Waypoint goal: {LED_goal_signal}")


if __name__ == '__main__':
    
    rclpy.init()

    node = led_manager(
        'led_manager_node', 
        namespace='safe_twist',
        cli_args=['--enable_debug'],
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
