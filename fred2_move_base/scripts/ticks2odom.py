#!/usr/bin/env python3

import rclpy
import threading
import sys
import transforms3d as tf3d     # angle manipulaton 

import fred2_move_base.scripts.debug as debug 
import fred2_move_base.scripts.parameters as params 
import fred2_move_base.scripts.qos as qos

from typing import List, Optional

from message_filters import Subscriber, ApproximateTimeSynchronizer

from rclpy.context import Context 
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.signals import SignalHandlerOptions

from tf2_ros import TransformBroadcaster

from math import pi, cos, sin

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Imu

# Node execution arguments 
debug_mode = '--debug' in sys.argv
publish_tf = '--publish-tf' in sys.argv         # For robot localization isn't necessary publish the TF




class OdometryNode(Node):

    left_wheels_ticks = 0               # Number of ticks recorded by the left wheel encoder
    right_wheels_ticks = 0              # Number of ticks recorded by the right wheel encoder

    last_left_ticks = 0                 # Previous left wheel tick count
    last_right_ticks = 0                # Previous right wheel tick count

    x_pos = 0                           # Current x position of the robot
    y_pos = 0                           # Current y position of the robot

    theta = 0                           # Current orientation angle (yaw) of the robot
    heading_offset = 0                  # Offset to adjust the robot's heading
    robot_heading = 0                   # Current heading obtained from the IMU

    delta_x = 0                         # Change in x position since the last calculation
    delta_y = 0                         # Change in y position since the last calculation
    delta_theta = 0                     # Change in orientation angle since the last calculation

    current_time = 0                    # Current time

    reset_odom = False                  # Flag indicating whether to reset odometry



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
        
        # --- Callback group for concurrent execution ---
        self.odom_callback_group = ReentrantCallbackGroup()   

        commum_qos_profile  = qos.general_configuration()
        imu_qos_profile = qos.imu_configuration()

        self.odom_pub = self.create_publisher(Odometry, '/odom', commum_qos_profile, callback_group=self.odom_callback_group)
   
        self.imu_sub = Subscriber(self, Imu, '/sensor/orientation/imu', qos_profile=imu_qos_profile)
        
        self.encoder_bl_sub = Subscriber(self, Int32, '/esp_back/power/status/distance/ticks/left')

        self.encoder_br_sub = Subscriber(self, Int32, '/esp_back/power/status/distance/ticks/right')
        
        self.reset_sub = self.create_subscription(Bool, 
                        '/odom/reset', 
                        lambda msg: self.odomReset_callback(node, msg), 
                        commum_qos_profile)
        
        self.sync = ApproximateTimeSynchronizer(
                    [self.imu_sub, self.encoder_bl_sub, self.encoder_br_sub],       # The topics you're syncing. Each one is a message_filters.Subscriber.
                    queue_size= 10,                                                 # Max number of recent messages to store per topic.
                    slop=0.3,                                                       # Allowed timestamp difference
                    allow_headerless=True
        )

        self.sync.registerCallback(self.synced_callback)         # Callback with syncronized callbacks
    
        params.odometry_config(self)


        self.last_time = self.get_clock().now()


        self.odom_broadcaster = TransformBroadcaster(self)
        # self.base_link_broadcaster = TransformBroadcaster(self)


        self.add_on_set_parameters_callback(params.odom_parameters_callback)


    def synced_callback(self, imu_msg:Imu, encoder_left:Int32, encoder_right:Int32):

        self.get_logger().info(
        f"Synced! IMU: {imu_msg.header.stamp.sec}.{imu_msg.header.stamp.nanosec}, "
        f"Encoder left: {encoder_left.stamp.sec}.{encoder_left.stamp.nanosec}"
        f"Encoder right: {encoder_right.stamp.sec}.{encoder_right.stamp.nanosec}"
    )
        
        quat = imu_msg.orientation
        imu_quat = [quat.w, quat.x, quat.y, quat.z]
        _, _, yaw = tf3d.euler.quat2euler(imu_quat)

        self.robot_heading = yaw

        self.left_wheels_ticks = encoder_left.data
        self.right_wheels_ticks = encoder_right.data


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

            self.reset_odom = False


        self.current_time = self.get_clock().now()

        self.elapse_time = (self.current_time - self.last_time).nanoseconds / 1e9


        # calculate the velocity 
        if self.elapse_time > 0: 

            self.linear_vel_x = self.delta_x / self.elapse_time
            self.linear_vel_y = self.delta_y / self.elapse_time
            self.angular_vel_theta = self.delta_theta / self.elapse_time
        


        self.odom_msg()
        


        if publish_tf or self.PUBLISH_ODOM_TF:
        
            self.odom_transform()



        self.last_left_ticks = self.left_wheels_ticks
        self.last_right_ticks = self.right_wheels_ticks

        self.last_time = self.current_time    


        if debug_mode or self.DEBUG: 

            debug.odometry(self)
            


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

        # base_link_tf = TransformStamped()

        # base_link_tf.header.stamp = self.get_clock().now().to_msg()
        # base_link_tf.header.frame_id = 'base_footprint'
        # base_link_tf.child_frame_id = 'base_link'
        # base_link_tf.transform.translation.x = 0.0
        # base_link_tf.transform.translation.y = 0.0
        # base_link_tf.transform.translation.z = self.BASE_LINK_OFFSET
        # base_link_tf.transform.rotation.x = 0.0 # no rotation
        # base_link_tf.transform.rotation.y = 0.0 
        # base_link_tf.transform.rotation.z = 0.0
        # base_link_tf.transform.rotation.w = 1.0

        # self.base_link_broadcaster.sendTransform(base_link_tf)


if __name__ == '__main__': 
    
    rclpy.init(args= None, signal_handler_options= SignalHandlerOptions.NO)

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
    executor = MultiThreadedExecutor(context=odom_context)
    executor.add_node(node)

    # create a separate thread for the callbacks and another for the main function 
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(10)

    try: 
        while rclpy.ok(): 
            node.calculate_odometry()
            rate.sleep()
        
    except KeyboardInterrupt:
        
        node.get_logger().warn(' ------------------------------------ DEACTIVATING NODE --------------------------------------')
        pass

    node.destroy_node()
    rclpy.shutdown()
    thread.join()


