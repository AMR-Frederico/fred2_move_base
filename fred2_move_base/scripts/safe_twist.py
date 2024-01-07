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

from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# Node execution arguments 
disable_ultrasonics = '--disable_ultrasonics' in sys.argv
debug_mode = '--debug' in sys.argv

# Parameters file (yaml)
node_path = '~/ros2_ws/src/fred2_move_base/config/move_base_params.yaml'
node_group = 'safe_twist'

smallest_reading = 1000

cmd_vel = Twist()

stop_by_obstacle = False 


class SafeTwistNode(Node):

    left_ultrasonic_distance = 1000
    right_ultrasonic_distance = 1000
    back_ultrasonic_distance = 1000

    user_abort_command = True
    abort_previous_flag = False
    
    robot_safety = False

    robot_vel = Twist()

    cmd_vel_safe = Twist()
    cmd_vel = Twist()


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
        
        self.create_subscription(Bool, 
                                 '/joy/controler/ps4/brake', 
                                 self.abort_callback, 
                                 qos_profile)
        
        self.safeVel_pub = self.create_publisher(Twist, 
                                                '/cmd_vel/safe', 
                                                qos_profile)

        self.userStop_pub = self.create_publisher(Bool, 
                                                  '/safety/abort/user_command', 
                                                  qos_profile)

        self.distanceStop_pub = self.create_publisher(Bool, 
                                                      '/safety/abort/colision_detection', 
                                                      qos_profile)
        
        self.ultrasonicDisabled_pub = self.create_publisher(Bool, 
                                                            '/safety/ultrasonic/disabled', 
                                                            qos_profile)

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
        self.SAFE_DISTANCE = self.get_parameter('safe_distance').value 
        self.MOTOR_BRAKE_FACTOR = self.get_parameter('motor_brake_factor').value
        self.MAX_LINEAR_SPEED = self.get_parameter('max_linear_speed').value
        self.MAX_ANGULAR_SPEED = self.get_parameter('max_angular_speed').value



    def rightUltrasonic_callback(self, distance): 
        
        self.right_ultrasonic_distance = distance.data
    



    def leftUltrasonic_callback(self, distance): 
        
        self.left_ultrasonic_distance = distance.data



    def backUltrasonic_callback(self, distance): 
        
        self.back_ultrasonic_distance = distance.data



    def odom_callback(self, msg): 
        
        self.robot_vel.linear.x = msg.twist.twist.linear.x
        self.robot_vel.angular.z = msg.twist.twist.angular.z
    



    def cmdVel_callback(self, velocity): 
        
        self.cmd_vel.linear.x = velocity.linear.x
        self.cmd_vel.angular.z = velocity.angular.z
    



    def abort_callback(self, user_command): 

        self.abort_flag = user_command.data 

        if (self.abort_flag > self.abort_previous_flag): 

            self.user_abort_command = not self.user_abort_command


        self.abort_previous_flag = self.abort_flag
        

        if self.user_abort_command: 

            self.cmd_vel_safe.linear.x = self.robot_vel.linear.x * self.MOTOR_BRAKE_FACTOR
            self.cmd_vel_safe.angular.z = self.robot_vel.angular.z * self.MOTOR_BRAKE_FACTOR

        self.robot_safety = not self.user_abort_command 


def main():

    global smallest_reading, stop_by_obstacle

    if disable_ultrasonics: 
        stop_by_obstacle = False
        braking_factor = 1

        node.get_logger().info('The ultrasonics sensors has been desable', once = True, throttle_duration_sec = 1.0)

    else: 

        if (node.right_ultrasonic_distance < node.SAFE_DISTANCE or
            node.left_ultrasonic_distance  < node.SAFE_DISTANCE or 
            node.back_ultrasonic_distance  < node.SAFE_DISTANCE): 

            stop_by_obstacle = True 

        else: 
            stop_by_obstacle = False


        if node.right_ultrasonic_distance < smallest_reading: 

            smallest_reading = node.right_ultrasonic_distance


        if node.left_ultrasonic_distance < smallest_reading: 

            smallest_reading = node.left_ultrasonic_distance


        if node.back_ultrasonic_distance < smallest_reading: 

            smallest_reading = node.back_ultrasonic_distance 


        braking_factor = smallest_reading/(2 * node.SAFE_DISTANCE)


        if braking_factor > 1: 

            braking_factor = 1
        

        if braking_factor < 0.5: 

            braking_factor = 0
        

    if node.robot_safety: 
        

        if stop_by_obstacle: 

            node.cmd_vel_safe.linear.x = node.robot_vel.linear.x * node.MOTOR_BRAKE_FACTOR
            node.cmd_vel_safe.angular.z = node.robot_vel.angular.z * node.MOTOR_BRAKE_FACTOR


        else: 

            node.cmd_vel_safe.linear.x = node.cmd_vel.linear.x * braking_factor
            node.cmd_vel_safe.angular.z = node.cmd_vel.angular.z * braking_factor 


        if (node.cmd_vel_safe.linear.x > node.MAX_LINEAR_SPEED): 

            node.cmd_vel_safe.linear.x = node.MAX_LINEAR_SPEED



        if (node.cmd_vel_safe.linear.x < - node.MAX_LINEAR_SPEED):

            node.cmd_vel_safe.linear.x = - node.MAX_LINEAR_SPEED 



        if (node.cmd_vel_safe.angular.z > node.MAX_ANGULAR_SPEED): 

            node.cmd_vel_safe.angular.z = node.MAX_ANGULAR_SPEED


        
        if (node.cmd_vel_safe.angular.z < - node.MAX_ANGULAR_SPEED): 

            node.cmd_vel_safe.angular.z = - node.MAX_ANGULAR_SPEED



    userStop_msg = Bool()
    userStop_msg.data = node.user_abort_command

    distanceStop_msg = Bool()
    distanceStop_msg.data = stop_by_obstacle

    ultrasonicDisabled_msg = Bool()
    ultrasonicDisabled_msg.data = disable_ultrasonics
    
    node.safeVel_pub.publish(node.cmd_vel_safe)
    node.userStop_pub.publish(userStop_msg)
    node.distanceStop_pub.publish(distanceStop_msg)
    node.ultrasonicDisabled_pub.publish(ultrasonicDisabled_msg)

    if debug_mode: 

        node.get_logger().info(f'Emergency mode -> user comand abort: {node.user_abort_command} | collision detected: {stop_by_obstacle} | ultrasonics disabled: {disable_ultrasonics}')
        node.get_logger().info(f'Ultrasonics -> left: {node.left_ultrasonic_distance} | right: {node.right_ultrasonic_distance} | Back: {node.back_ultrasonic_distance}')
        node.get_logger().info(f'Robot velocity -> linear: {node.robot_vel.linear.x} | angular: {node.robot_vel.angular.z}')
        node.get_logger().info(f'Velocity command -> linar: {cmd_vel.linear.x} | angular: {cmd_vel.angular.z} | braking_factor: {braking_factor}\n')
        
if __name__ == '__main__':
    # Create a custom context for single thread and real-time execution
    rclpy.init()

    safe_context = rclpy.Context()
    safe_context.init()
    safe_context.use_real_time = True
    

    node = SafeTwistNode(
        node_name='safe_twist',
        context=safe_context,
        cli_args=['--debug', '--disable_ultrasonics'],
        namespace='move_base',
        enable_rosout=False
    )

    # Make the execution in real time 
    executor = SingleThreadedExecutor(context=safe_context)
    executor.add_node(node)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(10)

    print(smallest_reading)

    try: 
        while rclpy.ok(): 
            main()
            rate.sleep()
        
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
