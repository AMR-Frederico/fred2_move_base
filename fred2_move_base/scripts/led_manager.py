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
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from rcl_interfaces.msg import Parameter, SetParametersResult
from rcl_interfaces.srv import GetParameters

from std_msgs.msg import Bool, Int16, Float32
from geometry_msgs.msg import PoseStamped, Pose2D


# Parameters file (yaml)
led_path = '~/ros2_ws/src/fred2_move_base/config/move_base_params.yaml'
led_group = 'led_manager'


# Check for cli_args 
debug_mode = '--debug' in sys.argv




class led_manager(Node): 
    

    led_color = Int16()
    led_debug = Int16()

    
    robot_state = -5    # Robot state, random initial value

    last_goal_reached = False

    user_stop_command = True
    collision_detected = False
    joy_connected = False

    ultrasonic_disabled = False

    goal_pose = Pose2D()

    led_goal_reached = False
    led_goal_signal = False

    last_robot_emergency = False

    odom_reset = False



    LED_ON_TIME = Duration(seconds=3) # It's also possible to specify nanoseconds



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


        # Load parameters from YAML file
        self.load_params(led_path, led_group)
        self.get_colors()
        

        self.start_time = self.get_clock().now()


        # quality protocol -> the node must not lose any message 
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            durability= QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=10, 
            liveliness=QoSLivelinessPolicy.AUTOMATIC
            
        )


        self.create_subscription(Bool,
                                 '/safety/abort/collision_alert', 
                                 self.collision_callback,
                                 5 )


        self.create_subscription(Bool, 
                                 '/safety/abort/user_command',
                                 self.manual_abort_callback,
                                 5 )
        

        self.create_subscription(Bool, 
                                 '/safety/ultrasonic/disabled', 
                                 self.ultrasonicStatus_callback, 
                                 5 )
        
        
        self.create_subscription(Bool, 
                                 '/joy/controller/connected', 
                                 self.joyConnected_callback, 
                                 5 )

        
        self.create_subscription(Int16,
                                 '/machine_states/robot_state',
                                 self.robot_state_callback, 
                                 qos_profile )
        

        self.create_subscription(PoseStamped,
                                 '/goal_manager/goal/current',
                                 self.goal_current_callback, 
                                 qos_profile )
        

        # self.create_subscription(Bool, 
        #                          '/goal_manager/goal/reached', 
        #                          self.goal_reached_callback, 
        #                          10 )
        
        self.create_subscription(Bool, 
                                 '/odom/reset', 
                                 self.odom_reset_callback, 
                                 5)
        

        self.ledColor_pub = self.create_publisher(Int16, 
                                                    '/cmd/led_strip/color', 
                                                    qos_profile )
        



        self.ledDebug_pub = self.create_publisher(Int16, 
                                                   '/cmd/led_strip/debug/color', 
                                                   5 )
        

        

        self.add_on_set_parameters_callback(self.parameters_callback)





    def parameters_callback(self, params):
        
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")


        if param.name == 'white':
            self.WHITE = param.value
    
  
        if param.name == 'blue':
            self.BLUE = param.value


        if param.name == 'yellow': 
            self.YELLOW = param.value


        if param.name == 'pink': 
            self.PINK = param.value
        

        if param.name == 'orange': 
            self.ORANGE = param.value


        if param.name == 'red': 
            self.RED = param.value


        if param.name == 'green': 
            self.GREEN = param.value


        if param.name == 'black': 
            self.BLACK = param.value


        if param.name == 'cyan': 
            self.CYAN = param.value


        if param.name == 'purple': 
            self.PURPLE = param.value    


        if param.name == 'light_green': 
            self.LIGHT_GREEN = param.value        

        
        if param.name == 'waypoint_goal': 
            self.WAYPOINT_GOAL = param.value


        if param.name == 'ghost_goal': 
            self.GHOST_GOAL = param.value



        return SetParametersResult(successful=True)
        
        

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
        self.CYAN = self.get_parameter('cyan').value
        self.PURPLE = self.get_parameter('purple').value
        self.LIGHT_GREEN = self.get_parameter('light_green').value    


        self.WAYPOINT_GOAL = self.get_parameter('waypoint_goal').value
        self.GHOST_GOAL = self.get_parameter('ghost_goal').value

        
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


            self.get_logger().info(f"Got global param ROBOT_MANUAL -> {self.ROBOT_MANUAL}")
            self.get_logger().info(f"Got global param ROBOT_AUTONOMOUS -> {self.ROBOT_AUTONOMOUS}")
            self.get_logger().info(f"Got global param ROBOT_IN GOAL -> {self.ROBOT_IN_GOAL}")
            self.get_logger().info(f"Got global param ROBOT_MISSION_COMPLETED: {self.ROBOT_MISSION_COMPLETED}")
            self.get_logger().info(f"Got global param ROBOT_EMERGENCY: {self.ROBOT_EMERGENCY}\n")



        except Exception as e:

            self.get_logger().warn("Service call failed %r" % (e,))




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

        self.user_stop_command = msg.data



    # Signal if the ultrasonics are disabled 
    def ultrasonicStatus_callback(self, msg): 

        if msg.data: 

            self.ultrasonic_disabled = True
        
        else: 

            self.ultrasonic_disabled = False



    # Current robot state
    def robot_state_callback(self, msg): 
        
        self.robot_state = msg.data





    def goal_current_callback(self, msg):
        
        self.goal_pose.x = msg.pose.position.x
        self.goal_pose.y = msg.pose.position.y
        self.goal_pose.theta = msg.pose.orientation.z


    def joyConnected_callback(self, msg): 
        
        self.joy_connected = msg.data



    def odom_reset_callback(self, msg): 
        
        self.odom_reset = msg.data


    # When the robot reaches the goal, analize if that is one the it shoud signal  
    # def goal_reached_callback(self, msg):

    #     self.goal_reached = msg.data

    #     if (self.goal_reached == True) and (self.last_goal_reached == False): 
            
    #         self.start_time = self.get_clock().now()
    #         self.led_goal_reached = True
        
    #     else: 

    #         self.led_goal_reached = False

    #     self.last_goal_reached = self.goal_reached


    #     # For keeping the signal on for a determined time
    #     if (self.get_clock().now() - self.start_time) > self.LED_ON_TIME: 
            
    #         self.led_goal_reached = False 

        

    #     if self.goal_pose.theta == self.WAYPOINT_GOAL: 
        
    #         self.led_goal_signal = True
        


        
        
 
    def led_manager(self):
        
        

        # Evaluate the sinalization for the goal reached 

        if self.robot_state == self.ROBOT_IN_GOAL: 

            
            self.led_goal_reached = True
            self.start_time = self.get_clock().now()


        # For keeping the signal on for a determined time
        if (self.get_clock().now() - self.start_time) > self.LED_ON_TIME: 
            
            self.led_goal_reached = False 
        

        if self.goal_pose.theta == self.WAYPOINT_GOAL: 
        
            self.led_goal_signal = True


        if self.goal_pose.theta == self.GHOST_GOAL: 
            
            self.led_goal_signal = False
            self.get_logger().warn('GHOST goal')

                

        #* Colors for the robot state: 

        if self.robot_state == self.ROBOT_EMERGENCY: 

            self.led_color.data = self.RED

        
        else: 


            if self.robot_state == self.ROBOT_AUTONOMOUS: 

                self.led_color.data = self.BLUE


                if (self.led_goal_reached == True) and (self.led_goal_signal == True):

                    self.get_logger().warn('GOAL SIGNAL -> LED ON')
                    self.led_color.data = self.GREEN



            if self.robot_state == self.ROBOT_MISSION_COMPLETED:

                self.led_color.data = self.CYAN




            if self.robot_state == self.ROBOT_MANUAL: 

                self.led_color.data = self.WHITE

            


        
        #* Colors for debug the robot states:
                
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


        if debug_mode: 

            self.get_logger().info(f"Color: {self.led_color.data}")
            self.get_logger().info(f"Debug color: {self.led_debug.data}")
            self.get_logger().info(f"Collision alert: {self.collision_detected}")
            self.get_logger().info(f"Stop command: {self.user_stop_command}")
            self.get_logger().info(f"Joy connected: {self.joy_connected}")
            self.get_logger().info(f"Ultrasonics disabled: {self.ultrasonic_disabled}")
            self.get_logger().info(f"Robot state: {self.robot_state}")
            self.get_logger().info(f"Waypoint goal: {self.led_goal_signal}\n")


if __name__ == '__main__':
    
    rclpy.init()

    node = led_manager(
        'led_manager', 
        namespace='move_base',
        cli_args=['--debug'],
        enable_rosout=False, 
        use_global_arguments=True, 
    )

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(1)

    try: 
        while rclpy.ok(): 
            
            node.led_manager()
            rate.sleep()
        
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
