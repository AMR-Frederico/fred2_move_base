#!/usr/bin/env python3

import rclpy
import threading
import yaml 
import sys
import os
import transforms3d as tf3d     # angle manipulaton 

from typing import List, Optional

from rclpy.node import Node
from rclpy.context import Context 
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from tf2_ros import TransformBroadcaster

from math import pi, cos, sin

from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

# Parameters file (yaml)
node_path = '~/ros2_ws/src/fred2_move_base/config/move_base_params.yaml'
node_group = 'odometry'

# Node execution arguments 
debug_mode = '--debug' in sys.argv

publish_tf = '--publish-tf' in sys.argv         # For robot localization isn't necessary publish the TF

class OdometryNode(Node):

    left_wheels_ticks = 0
    right_wheels_ticks = 0

    last_left_ticks = 0
    last_right_ticks = 0

    x_pos = 0
    y_pos = 0

    theta = 0
    heading_offset = 0
    robot_heading = 0

    delta_x = 0
    delta_y = 0
    delta_theta = 0

    current_time = 0

    reset_odom = False

    
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


        # quality protocol -> the node must not lose any message 
        imu_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=5
        )

        self.create_subscription(Int32, 
                                 '/power/status/distance/ticks/left', 
                                 self.ticksLeft_callback,
                                 qos_profile)

        self.create_subscription(Int32, 
                                 '/power/status/distance/ticks/right', 
                                 self.ticksRight_callback, 
                                 qos_profile)
        
        self.create_subscription(Imu, 
                                 '/sensor/orientation/imu', 
                                 self.heading_callback, 
                                 imu_qos_profile)
        
        self.create_subscription(Bool, 
                                 '/odom/reset', 
                                 self.odomReset_callback, 
                                 qos_profile)
        

        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)


        self.load_params(node_path, node_group)
        self.get_params()


        self.last_time = self.get_clock().now()
        
        
        self.add_on_set_parameters_callback(self.parameters_callback)


        self.odom_broadcaster = TransformBroadcaster(self)
        self.base_link_broadcaster = TransformBroadcaster(self)





    def parameters_callback(self, params):
        
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")



        if param.name == 'wheels_track':
            self.WHEELS_TRACK = param.value
    
  
        if param.name == 'wheels_radius':
            self.WHEELS_RADIUS = param.value


        if param.name == 'ticks_per_revolution': 
            self.TICKS_PER_TURN = param.value


        if param.name == 'base_link_offset': 
            self.BASE_LINK_OFFSET = param.value
        


        return SetParametersResult(successful=True)




    def ticksLeft_callback(self, ticks_msg): 

        self.left_wheels_ticks = ticks_msg.data




    def ticksRight_callback(self, ticks_msg): 
        
        self.right_wheels_ticks = ticks_msg.data




    def heading_callback(self, imu_msg): 
        
        self.robot_quaternion = imu_msg

        # get the yaw angle
        self.robot_heading = tf3d.euler.quat2euler([self.robot_quaternion.orientation.w, 
                                                    self.robot_quaternion.orientation.x, 
                                                    self.robot_quaternion.orientation.y, 
                                                    self.robot_quaternion.orientation.z])[2]
        


    def odomReset_callback(self, reset_msg):
        
        self.reset_odom = reset_msg.data



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

        self.WHEELS_TRACK = self.get_parameter('wheels_track').value 
        self.WHEELS_RADIUS = self.get_parameter('wheels_radius').value
        self.TICKS_PER_TURN = self.get_parameter('ticks_per_revolution').value
        self.BASE_LINK_OFFSET = self.get_parameter('base_link_offset').value



    def calculate_odometry(self): 

        self.delta_left_ticks = self.left_wheels_ticks - self.last_left_ticks
        self.delta_right_ticks = self.right_wheels_ticks - self.last_right_ticks


        # distance traveled by each wheel  
        self.distance_left = (2 * pi * self.WHEELS_RADIUS * self.delta_left_ticks) / self.TICKS_PER_TURN
        self.distance_right = (2 * pi * self.WHEELS_RADIUS * self.delta_right_ticks) / self.TICKS_PER_TURN

        # the average distance traveled by the robot is the average of the distances traveled by each wheels
        self.robot_moviment_distance = (self.distance_left + self.distance_right) / 2

        # the change in orientation (yaw angle) of the robot, calculated using the difference in distances traveled by each and the wheelbase
        self.delta_theta = (self.distance_right - self.distance_left) / self.WHEELS_TRACK

        # assume the robot is moving in a straight line
        if self.distance_left == self.distance_right: 

            self.delta_x = self.robot_moviment_distance * cos(self.theta)
            self.delta_y = self.robot_moviment_distance * sin(self.theta)
        
        else: 

            self.turning_radius = self.robot_moviment_distance / self.delta_theta

            # icc -> Instantaneous Center of Curvature
            self.icc_x = self.x_pos - self.turning_radius * sin(self.theta)
            self.icc_y = self.y_pos + self.turning_radius * cos(self.theta)

            self.delta_x = cos(self.delta_theta) * (self.x_pos - self.icc_x) - sin(self.delta_theta) * (self.y_pos - self.icc_y) + self.icc_x - self.x_pos
            self.delta_y = sin(self.delta_theta) * (self.x_pos - self.icc_x) + cos(self.delta_theta) * (self.y_pos - self.icc_y) + self.icc_y - self.y_pos

        self.x_pos += self.delta_x
        self.y_pos += self.delta_y

        self.theta = self.robot_heading - self.heading_offset
        # self.theta = (self.theta + self.delta_theta) % (2 * pi) # determinate robot heading by encoder 

        if self.reset_odom: 
            self.x_pos = 0.0
            self.y_pos = 0.0
            self.heading_offset = self.robot_heading

            self.get_logger().info('Odometry reset')


        self.current_time = self.get_clock().now()

        self.elapse_time = (self.current_time - self.last_time).nanoseconds / 1e9


        # calculate the velocity 
        if self.elapse_time > 0: 

            self.linear_vel_x = self.delta_x / self.elapse_time
            self.linear_vel_y = self.delta_y / self.elapse_time
            self.angular_vel_theta = self.delta_theta / self.elapse_time
        


        self.odom_msg()
        


        if publish_tf:
        
            self.odom_transform()



        self.last_left_ticks = self.left_wheels_ticks
        self.last_right_ticks = self.right_wheels_ticks

        self.last_time = self.current_time    


        if debug_mode: 
            node.get_logger().info(f'Position -> x: {self.x_pos} | y: {self.y_pos}')
            node.get_logger().info(f'Velocity -> x: {self.linear_vel_x} | y: {self.linear_vel_y} | theta: {self.angular_vel_theta}')
            node.get_logger().info(f'Ticks -> left: {self.left_wheels_ticks} | right: {self.right_wheels_ticks} \n')



    def odom_msg(self): 

        odom_msg = Odometry()

        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        odom_msg.pose.pose.position.x = self.x_pos
        odom_msg.pose.pose.position.y = self.y_pos
        odom_msg.pose.pose.position.z = 0.0

        # Calculate quaternion from Euler angles
        self.odom_quat = tf3d.euler.euler2quat(0, 0, self.theta)

        odom_msg.pose.pose.orientation.x = self.odom_quat[1]
        odom_msg.pose.pose.orientation.y = self.odom_quat[2]
        odom_msg.pose.pose.orientation.z = self.odom_quat[3]
        odom_msg.pose.pose.orientation.w = self.odom_quat[0]

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_vel_theta

        odom_msg.twist.twist.linear.x = self.linear_vel_x
        odom_msg.twist.twist.linear.y = self.linear_vel_y
        odom_msg.twist.twist.linear.z = 0.0

        self.odom_pub.publish(odom_msg)



    def odom_transform(self): 

        odom_tf = TransformStamped()

        odom_tf.header.stamp = self.get_clock().now().to_msg()
        odom_tf.header.frame_id = 'odom'
        odom_tf.child_frame_id = 'base_footprint'
        odom_tf.transform.translation.x = self.x_pos  
        odom_tf.transform.translation.y = self.y_pos 
        odom_tf.transform.translation.z = 0.0


        odom_tf.transform.rotation.x = self.odom_quat[1]
        odom_tf.transform.rotation.y = self.odom_quat[2]
        odom_tf.transform.rotation.z = self.odom_quat[3] 
        odom_tf.transform.rotation.w = self.odom_quat[0]

        self.odom_broadcaster.sendTransform(odom_tf)

        base_link_tf = TransformStamped()

        base_link_tf.header.stamp = self.get_clock().now().to_msg()
        base_link_tf.header.frame_id = 'base_footprint'
        base_link_tf.child_frame_id = 'base_link'
        base_link_tf.transform.translation.x = 0.0
        base_link_tf.transform.translation.y = 0.0
        base_link_tf.transform.translation.z = self.BASE_LINK_OFFSET
        base_link_tf.transform.rotation.x = 0.0 # no rotation
        base_link_tf.transform.rotation.y = 0.0 
        base_link_tf.transform.rotation.z = 0.0
        base_link_tf.transform.rotation.w = 1.0

        self.base_link_broadcaster.sendTransform(base_link_tf)


if __name__ == '__main__': 
    
    rclpy.init()

    odom_context = rclpy.Context()
    odom_context.init()
    odom_context.use_real_time = True

    node = OdometryNode(
        node_name='odometry',
        context=odom_context,
        cli_args=['--debug', '--publish-tf'],
        namespace='move_base',
        enable_rosout=False
    )

    # Make the execution in real time 
    executor = SingleThreadedExecutor(context=odom_context)
    executor.add_node(node)

    # create a separate thread for the callbacks and another for the main function 
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(1)

    try: 
        while rclpy.ok(): 
            node.calculate_odometry()
            rate.sleep()
        
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


