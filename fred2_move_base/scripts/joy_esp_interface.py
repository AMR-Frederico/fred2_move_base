#!/usr/bin/env python3

import rclpy
import threading
import sys


from typing import List, Optional

from rclpy.context import Context 
from rclpy.node import Node, ParameterDescriptor
from rclpy.parameter import Parameter, ParameterType
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy


from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters

from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Parameters file (yaml)
node_path = '/home/ubuntu/ros2_ws/src/fred2_move_base/config/move_base_params.yaml'
node_group = 'joy_esp_interface'


# Check for cli_args 
debug_mode = '--debug' in sys.argv

class JoyInterfaceNode(Node):

    joy_vel_angular = 0
    joy_vel_linear = 0

    cmd_vel = Twist()

    last_reset = 0 
    last_switch = 0

    reset_odom = False
    switch_mode = False

    manual_mode = True

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
            durability= QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=10, 
            liveliness=QoSLivelinessPolicy.AUTOMATIC
            
        )

        self.create_subscription(Joy, 
                                 '/joy/controller/ps4', 
                                 self.joy_callback, 
                                 10)
        

        
        self.create_subscription(Int16, 
                            '/machine_states/robot_state', 
                            self.manualMode_callback, 
                            qos_profile)



        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)      

        self.resetOdom_pub = self.create_publisher(Bool, '/odom/reset', qos_profile)

        self.switchMode_pub = self.create_publisher(Bool, '/joy/machine_states/switch_mode', qos_profile) 


        # load params from the config file 
        self.load_params()
        self.get_params()


        self.add_on_set_parameters_callback(self.parameters_callback)

        
        self.last_joy_command_time = self.get_clock().now()


    def load_params(self):
        # Declare parameters for joystick and velocity limits, as well as debug settings
        self.declare_parameters(
            namespace='',
            parameters=[
                ('drift_analog_tolerance', None, ParameterDescriptor(description='Tolerance for analog drift in joystick', type=ParameterType.PARAMETER_INTEGER)),
                ('max_value_controller', None, ParameterDescriptor(description='Maximum value for joystick controller', type=ParameterType.PARAMETER_INTEGER)),
                ('max_vel_joy_angular', None, ParameterDescriptor(description='Maximum angular velocity from joystick', type=ParameterType.PARAMETER_DOUBLE)),
                ('max_vel_joy_linear', None, ParameterDescriptor(description='Maximum linear velocity from joystick', type=ParameterType.PARAMETER_DOUBLE)),
                ('debug', None, ParameterDescriptor(description='Enable debug prints for troubleshooting', type=ParameterType.PARAMETER_BOOL)),
                ('unit_test', None, ParameterDescriptor(description='Enable unit testing mode', type=ParameterType.PARAMETER_BOOL))
            ]
        )

        self.get_logger().info('All parameters successfully declared')


    def parameters_callback(self, params):
        
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")


        if param.name == 'max_vel_joy_linear':
            self.MAX_VEL_JOY_LINEAR = param.value
    
  
        if param.name == 'max_vel_joy_angular':
            self.MAX_VEL_JOY_ANGULAR = param.value


        if param.name == 'max_value_controller': 
            self.MAX_VALUE_CONTROLLER = param.value


        if param.name == 'drift_analog_tolerance': 
            self.DRIFT_ANALOG_TOLERANCE = param.value
    

        if param.name == 'debug': 
            self.DEBUG = param.value 


        if param.name == 'unit_test': 
            self.UNIT_TEST = param.value 


        return SetParametersResult(successful=True)
    


    def get_params(self):

        self.MAX_VEL_JOY_LINEAR = self.get_parameter('max_vel_joy_linear').value
        self.MAX_VEL_JOY_ANGULAR = self.get_parameter('max_vel_joy_angular').value
        self.MAX_VALUE_CONTROLLER = self.get_parameter('max_value_controller').value
        self.DRIFT_ANALOG_TOLERANCE = self.get_parameter('drift_analog_tolerance').value
        self.UNIT_TEST = self.get_parameter('unit_test').value
        self.DEBUG = self.get_parameter('debug').value


        # Allows it to run without the machine states 
        if self.UNIT_TEST: 
            
            self.manual_mode = True
        
        else: 
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



    def joy_callback(self, joy_msg): 

        # analog 

        if abs(joy_msg.axes[0]) > self.DRIFT_ANALOG_TOLERANCE: 

            self.joy_vel_linear = joy_msg.axes[0]

        else: 

            self.joy_vel_linear = 0



        # analog 

        if abs(joy_msg.axes[2]) > self.DRIFT_ANALOG_TOLERANCE: 

            self.joy_vel_angular = joy_msg.axes[2]

        else: 

            self.joy_vel_angular = 0 



        # buttons 
        reset_button = joy_msg.buttons[1]
        switch_button = joy_msg.buttons[2]
    


        if reset_button > self.last_reset: 

            self.get_logger().info('Reset odometry')
            self.reset_robot_odom(True)

        else: 

            self.reset_robot_odom(False)
      
        


        if switch_button > self.last_switch: 
            
            self.get_logger().info('Switch mode')
            self.switch_robot_mode(True)

        else: 

            self.switch_robot_mode(False)


        
        self.last_joy_command_time = self.get_clock().now()
        
        self.last_switch = switch_button
        self.last_reset = reset_button




    def switch_robot_mode(self, status): 

        
        change_mode = Bool()
        change_mode.data = status        
        self.switchMode_pub.publish(change_mode)



    def reset_robot_odom(self, status): 


        # reset the state of the main machine states, for the initial one 
        reset_msg = Bool()
        reset_msg.data = status
        self.resetOdom_pub.publish(reset_msg)

    

    def manualMode_callback(self, msg):

        robot_state = msg.data

        if robot_state == self.ROBOT_MANUAL: 

            self.manual_mode = True
        
        else: 

            self.manual_mode = False
    




    def main(self): 

        current_time = self.get_clock().now()

        vel_linear = self.joy_vel_linear * (self.MAX_VEL_JOY_LINEAR / self.MAX_VALUE_CONTROLLER)
        vel_angular = self.joy_vel_angular * (self.MAX_VEL_JOY_ANGULAR / self.MAX_VALUE_CONTROLLER)  

        if (current_time - self.last_joy_command_time).nanoseconds > 1.5e8  and self.manual_mode: 

            vel_linear = 0.0
            vel_angular = 0.0
        

        # cmd vel msg 
        self.cmd_vel.linear.x = vel_linear
        self.cmd_vel.angular.z = -1 * vel_angular   # correção de sentido


        if self.manual_mode: 

            self.vel_pub.publish(self.cmd_vel)



        
        if debug_mode or self.DEBUG:
             
            self.get_logger().info(f'Velocity -> linear:{vel_linear} | angular:{vel_angular}\n')
            self.get_logger().info(f'Reset odometry -> {self.reset_odom}')
            self.get_logger().info(f'Switch mode -> {self.switch_mode}')
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

    rate = node.create_rate(7)

    try: 
        while rclpy.ok(): 
            node.main()
            rate.sleep()
        
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
