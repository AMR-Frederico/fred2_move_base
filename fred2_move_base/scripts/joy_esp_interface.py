#!/usr/bin/env python3

import rclpy
import threading
import yaml
import sys
import os

from typing import List, Optional

from rclpy.node import Node
from rclpy.context import Context 
from rclpy.parameter import Parameter
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy

from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Twist

# Parameters file (yaml)
node_path = '~/ros2_ws/src/fred2_move_base/config/move_base_params.yaml'
node_group = 'joy_esp_interface'

cmd_vel = Twist()

last_reset_odom = 0

change_mode = Bool()
last_mode = False

# Check for cli_args 
debug_mode = '--debug' in sys.argv

class JoyInterfaceNode(Node):

    # controller buttons
    controler_buttons = {"square": None,
                         "circle": None,
                         "triangule": None,
                         "x": None,
                         "R2": None,
                         "L2": None,
                         "L_Y":0,
                         "R_X":0}
    
    # robot state
    manual_mode = False
    switch_mode = False

    # odometry
    reset_odom = 0
    
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

        self.create_subscription(Int16, 
                                 '/joy/controler/ps4/cmd_vel/linear', 
                                 self.velLinear_callback, 
                                 qos_profile)
        
        self.create_subscription(Int16, 
                                 '/joy/controler/ps4/cmd_vel/angular', 
                                 self.velAngular_callback, 
                                 qos_profile)
        
        # self.create_subscription(Bool, 
        #                          '/machine_state/control_mode/manual', 
        #                          self.manualMode_callback, 
        #                          qos_profile)

        self.create_subscription(Int16, 
                                 '/joy/controler/ps4/circle', 
                                 self.resetOdom_callback, 
                                 qos_profile)

        self.create_subscription(Int16, 
                                 '/joy/controler/ps4/triangle', 
                                 self.switchMode_callback, 
                                 qos_profile)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)      

        self.resetOdom_pub = self.create_publisher(Bool, '/odom/reset', 1)

        self.goalsReset_pub = self.create_publisher(Bool, '/goal_manager/goal/reset', 1)

        self.switchMode_pub = self.create_publisher(Bool, '/joy/machine_states/switch_mode', 1) 

        self.missionCompleted_pub = self.create_publisher(Bool, '/goal_manager/goal/mission_completed', 1)


        # load params from the config file 
        self.load_params(node_path, node_group)
        self.get_params()

    
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
        self.MAX_SPEED_JOY_LINEAR = self.get_parameter('max_speed_joy_linear').value
        self.MAX_SPEED_JOY_ANGULAR = self.get_parameter('max_speed_joy_angular').value
        self.MAX_VALUE_CONTROLLER = self.get_parameter('max_value_controller').value
        self.DRIFT_ANALOG_TOLERANCE = self.get_parameter('drift_analog_tolerance').value


    # get linear vel, if is it above min threshold
    def velLinear_callback(self, vel_msg): 
        
        if(abs(vel_msg.data) > self.DRIFT_ANALOG_TOLERANCE): 
            
            self.controler_buttons['L_Y'] = vel_msg.data
            
        else:
            
            self.controler_buttons['L_Y'] = 0
    


    # get angular vel, if is it above min threshold
    def velAngular_callback(self, vel_msg): 

        if(abs(vel_msg.data) > self.DRIFT_ANALOG_TOLERANCE):
            
            self.controler_buttons['R_X'] = vel_msg.data
        
        else:
            
            self.controler_buttons['R_X'] = 0



    # def manualMode_callback(self, manual_msg):

    #     self.manual_mode = manual_msg.data



    def resetOdom_callback(self, reset_msg): 

        self.reset_odom = reset_msg.data



    def switchMode_callback(self, mode_msg): 

        self.switch_mode = mode_msg.data


def main():

    global last_reset_odom, last_mode
    
    #* speed control (anolog buttons)
    # only send comands if manual mode is on 
    vel_angular = 0
    vel_linear = 0

    # rule of three equating the maximum speed of the joy with that of the robot
    vel_angular = node.controler_buttons['R_X'] * (node.MAX_SPEED_JOY_ANGULAR / node.MAX_VALUE_CONTROLLER)
    vel_linear = node.controler_buttons['L_Y'] * (node.MAX_SPEED_JOY_LINEAR / node.MAX_VALUE_CONTROLLER)
    

    cmd_vel.linear.x = vel_linear
    cmd_vel.angular.z = -1 * vel_angular #? Correção de sentido? 
    

    if node.manual_mode:
        
        node.vel_pub.publish(cmd_vel)

    
    #* reset odometry (circle buttom)
    odom_reset = (node.reset_odom > last_reset_odom)
    last_reset_odom = node.reset_odom


    # reset the state of the main machine states, for the initial one 
    if odom_reset: 

        missionCompleted_msg = Bool()
        missionCompleted_msg.data = False
        node.missionCompleted_pub.publish(missionCompleted_msg)
    

    reset_msg = Bool()
    reset_msg.data = odom_reset


    node.resetOdom_pub.publish(reset_msg)
    node.goalsReset_pub.publish(reset_msg)


    #* switch mode (triangle buttom)
    change_mode.data = node.switch_mode > last_mode
    
    last_mode = node.switch_mode 


    node.switchMode_pub.publish(change_mode)

    
    if debug_mode: 
        node.get_logger().info(f'Velocity -> linear:{vel_linear} | angular:{vel_angular}\n')
        node.get_logger().info(f'Reset odometry -> {odom_reset}')
        node.get_logger().info(f'Switch mode -> {change_mode.data}')
        # node.get_logger().info(f'Manual mode -> {node.manual_mode}')


if __name__ == '__main__': 
    
    rclpy.init()

    node = JoyInterfaceNode(
        node_name='joy_esp_interface',
        cli_args=['--debug'],
        namespace='move_base',
        enable_rosout=False)

    # separete theread for main function 
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
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