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

from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters

from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Twist

# Parameters file (yaml)
node_path = '~/ros2_ws/src/fred2_move_base/config/move_base_params.yaml'
node_group = 'joy_esp_interface'


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


    cmd_vel = Twist()

    last_reset_odom = 0

    change_mode = Bool()
    last_mode = False

    # starts with randon value 
    ROBOT_MANUAL = 1000
    ROBOT_AUTONOMOUS = 1000
    ROBOT_IN_GOAL = 1000
    ROBOT_MISSION_COMPLETED = 1000
    ROBOT_EMERGENCY = 1000

    
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
                                 '/joy/controller/ps4/cmd_vel/linear', 
                                 self.velLinear_callback, 
                                 qos_profile)
        
        self.create_subscription(Int16, 
                                 '/joy/controller/ps4/cmd_vel/angular', 
                                 self.velAngular_callback, 
                                 qos_profile)
        
        self.create_subscription(Int16, 
                                 '/machine_states/robot_state', 
                                 self.manualMode_callback, 
                                 qos_profile)

        self.create_subscription(Int16, 
                                 '/joy/controller/ps4/circle', 
                                 self.resetOdom_callback, 
                                 qos_profile)

        self.create_subscription(Int16, 
                                 '/joy/controller/ps4/triangle', 
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


        self.add_on_set_parameters_callback(self.parameters_callback)

        
        self.last_angular_vel_command_time = self.get_clock().now()
        self.last_linear_vel_command_time = self.get_clock().now()


    def parameters_callback(self, params):
        
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")



        if param.name == 'max_speed_joy_linear':
            self.MAX_SPEED_JOY_LINEAR = param.value
    
  
        if param.name == 'max_speed_joy_angular':
            self.MAX_SPEED_JOY_ANGULAR = param.value


        if param.name == 'max_value_controller': 
            self.MAX_VALUE_CONTROLLER = param.value


        if param.name == 'drift_analog_tolerance': 
            self.DRIFT_ANALOG_TOLERANCE = param.value
    


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

        self.MAX_SPEED_JOY_LINEAR = self.get_parameter('max_speed_joy_linear').value
        self.MAX_SPEED_JOY_ANGULAR = self.get_parameter('max_speed_joy_angular').value
        self.MAX_VALUE_CONTROLLER = self.get_parameter('max_value_controller').value
        self.DRIFT_ANALOG_TOLERANCE = self.get_parameter('drift_analog_tolerance').value

        # Get global params 

        self.client = self.create_client(GetParameters, '/machine_states/main_robot/get_parameters')
        self.client.wait_for_service()

        request = GetParameters.Request()
        request.names = ['manual', 'autonomous', 'in_goal', 'mission_completed', 'emergency']

        future = self.client.call_async(request)
        future.add_done_callback(self.callback_global_param)


    
    def callback_global_param(self, future):


        try:

            result = future.result()

            self.ROBOT_MANUAL = result.values[0].integer_value
            self.ROBOT_AUTONOMOUS = result.values[1].integer_value
            self.ROBOT_IN_GOAL = result.values[2].integer_value
            self.ROBOT_MISSION_COMPLETED = result.values[3].integer_value
            self.ROBOT_EMERGENCY = result.values[4].integer_value


            self.get_logger().info(f"\nGot global param ROBOT_MANUAL -> {self.ROBOT_MANUAL}")
            self.get_logger().info(f"Got global param ROBOT_AUTONOMOUS -> {self.ROBOT_AUTONOMOUS}")
            self.get_logger().info(f"Got global param ROBOT_IN GOAL -> {self.ROBOT_IN_GOAL}")
            self.get_logger().info(f"Got global param ROBOT_MISSION_COMPLETED: {self.ROBOT_MISSION_COMPLETED}")
            self.get_logger().info(f"Got global param ROBOT_EMERGENCY: {self.ROBOT_EMERGENCY}\n")



        except Exception as e:

            self.get_logger().warn("Service call failed %r" % (e,))






    # get linear vel, if is it above min threshold
    def velLinear_callback(self, vel_msg): 
        
        if(abs(vel_msg.data) > self.DRIFT_ANALOG_TOLERANCE): 
            
            self.controler_buttons['L_Y'] = vel_msg.data
            
        else:
            
            self.controler_buttons['L_Y'] = 0
        
        self.last_angular_vel_command_time = self.get_clock().now()
    


    # get angular vel, if is it above min threshold
    def velAngular_callback(self, vel_msg): 

        if(abs(vel_msg.data) > self.DRIFT_ANALOG_TOLERANCE):
            
            self.controler_buttons['R_X'] = vel_msg.data
        
        else:
            
            self.controler_buttons['R_X'] = 0
        
        self.last_linear_vel_command_time = self.get_clock().now()



    def manualMode_callback(self, msg):

        robot_state = msg.data

        if robot_state == self.ROBOT_MANUAL: 

            self.manual_mode = True
        
        else: 

            self.manual_mode = False



    def resetOdom_callback(self, reset_msg): 

        self.reset_odom = reset_msg.data



    def switchMode_callback(self, mode_msg): 

        self.switch_mode = mode_msg.data


    def main(self):

        current_time = self.get_clock().now()
        
        #* speed control (anolog buttons)
        # only send comands if manual mode is on 
        vel_angular = 0
        vel_linear = 0

        # rule of three equating the maximum speed of the joy with that of the robot
        vel_angular = self.controler_buttons['R_X'] * (self.MAX_SPEED_JOY_ANGULAR / self.MAX_VALUE_CONTROLLER)
        vel_linear = self.controler_buttons['L_Y'] * (self.MAX_SPEED_JOY_LINEAR / self.MAX_VALUE_CONTROLLER)
        

        # Check if the last linear velocity command message was received within the last 2 seconds
        if (current_time - self.last_linear_vel_command_time).nanoseconds > 2e9 and self.manual_mode:
            
            self.get_logger().warn('Velocity angular command reset due to a timeout (no message received within the last 2 seconds).')
            vel_angular = 0.0


        # Check if the last angular velocity command message was received within the last 2 seconds
        if (current_time - self.last_angular_vel_command_time).nanoseconds > 2e9 and self.manual_mode:
            
            self.get_logger().warn('Velocity linear command reset due to a timeout (no message received within the last 2 seconds).')
            vel_linear = 0.0



        self.cmd_vel.linear.x = vel_linear
        self.cmd_vel.angular.z = -1 * vel_angular #? Correção de sentido? 

        
        
        if self.manual_mode:
            
            self.vel_pub.publish(self.cmd_vel)


        
        #* reset odometry (circle buttom)
        odom_reset = (self.reset_odom > self.last_reset_odom)
        self.last_reset_odom = self.reset_odom


        # reset the state of the main machine states, for the initial one 
        if odom_reset: 

            missionCompleted_msg = Bool()
            missionCompleted_msg.data = False
            self.missionCompleted_pub.publish(missionCompleted_msg)

            self.get_logger().info('Reset odometry')
        

        reset_msg = Bool()
        reset_msg.data = odom_reset


        self.resetOdom_pub.publish(reset_msg)
        self.goalsReset_pub.publish(reset_msg)


        #* switch mode (triangle buttom)
        self.change_mode.data = self.switch_mode > self.last_mode

        if self.change_mode.data: 
            self.get_logger().info('Switch mode')
        
        self.last_mode = self.switch_mode 


        self.switchMode_pub.publish(self.change_mode)

        
        if debug_mode: 
            self.get_logger().info(f'Velocity -> linear:{vel_linear} | angular:{vel_angular}\n')
            self.get_logger().info(f'Reset odometry -> {odom_reset}')
            self.get_logger().info(f'Switch mode -> {self.change_mode.data}')
            self.get_logger().info(f'Manual mode -> {self.manual_mode}')


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

    rate = node.create_rate(1)

    try: 
        while rclpy.ok(): 
            node.main()
            rate.sleep()
        
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()