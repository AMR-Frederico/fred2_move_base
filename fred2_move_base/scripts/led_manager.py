#!/usr/bin/env python3

import rclpy
import yaml
import os

from typing import Any, List, Optional
from rcl_interfaces.msg import Parameter, ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import Bool, Int16

led_path = '~/ros2_ws/src/fred2_move_base/config/move_base_params.yaml'
led_group = 'led_manager'

collision_detected = False

user_stop_command = True
is_emergency_stop = False
last_stop_command = False

robot_state = -5 

class led_manager(Node): 
    
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
        
        self.robot_state = self.create_subscription(
                                    Int16, 
                                    self.robot_state_callback, 
                                    10 )

    
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
 
def main():

    rclpy.init()

    # Put the parameters initial state


    node = led_manager(
        'led_manager_node', 
        namespace='safe_twist',
        cli_args=['--enable_debug'],
        enable_rosout=False, 
    )

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
