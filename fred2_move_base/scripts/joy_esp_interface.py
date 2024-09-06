#!/usr/bin/env python3

import rclpy
import threading
import sys
import os
import datetime
from pathlib import Path

from ament_index_python import get_package_share_directory

import fred2_move_base.scripts.debug as debug
import fred2_move_base.scripts.parameters as params 
import fred2_move_base.scripts.publishers as publishers 
import fred2_move_base.scripts.subscribers as subscribers 

from typing import List

from rclpy.context import Context 
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


# Check for cli_args 
debug_mode = '--debug' in sys.argv
operation_time = str(datetime.datetime.now())

class JoyInterfaceNode(Node):

    joy_vel_angular = 0
    joy_vel_linear = 0

    cmd_vel = Twist()

    last_reset = 0 
    last_switch = 0
    last_save_ghost_point_button = 0
    last_save_goal_point_button = 0

    reset_odom = False
    switch_mode = False
    save_ghost_point = False
    save_goal_point = False

    position_x = 999
    position_y = 999
    position_z = 999


    manual_mode = True

    # starts with randon value 
    ROBOT_MANUAL = 1000
    ROBOT_AUTONOMOUS = 1000
    ROBOT_INIT = 1000
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
    
        subscribers.joy_config(self)
        publishers.joy_config(self)
        params.joy_config(self)


        self.add_on_set_parameters_callback(params.joy_params_callback)

        
        self.last_joy_command_time = self.get_clock().now()


    
    def joy_command(self):


 
        # Process linear velocity command from joystick analog input
        if abs(self.joy_msg.axes[0]) > self.DRIFT_ANALOG_TOLERANCE: 

            self.joy_vel_linear = self.joy_msg.axes[0]

        else: 

            self.joy_vel_linear = 0                     # If joystick analog input is below tolerance, set linear velocity to 0


        # Process angular velocity command from joystick analog input
        if abs(self.joy_msg.axes[2]) > self.DRIFT_ANALOG_TOLERANCE: 

            self.joy_vel_angular = self.joy_msg.axes[2]      

        else: 

            self.joy_vel_angular = 0                    # If joystick analog input is below tolerance, set angular velocity to 0



        # Process button commands
        reset_button = self.joy_msg.buttons[1]               # CIRCLE -> Button for resetting odometry
        switch_button = self.joy_msg.buttons[2]              # TRIANGLE -> Button for switching mode
        save_ghost_point_button = self.joy_msg.buttons[2]    # ARROW_DOWN
        save_goal_point_button = self.joy_msg.buttons[3]     # ARROW_UP
    

         # Check if reset button is pressed
        if reset_button > self.last_reset: 

            self.get_logger().info('Reset odometry')
            self.reset_robot_odom(True)

        else: 

            self.reset_robot_odom(False)                # If reset button is not pressed, do not reset odom   
      
        

        # Check if switch button is pressed
        if switch_button > self.last_switch: 
            
            self.get_logger().info('Switch mode')
            self.switch_robot_mode(True)

        else: 

            self.switch_robot_mode(False)               # If switch button is not pressed, do not switch mode


        # save points joy btn triggered
        if save_ghost_point_button > self.last_save_ghost_point_button:
            self.save_point(False)

        if save_goal_point_button > self.last_save_goal_point_button:
            self.save_point(True)


         # Update time of last joystick command
        self.last_joy_command_time = self.get_clock().now()
        
         # Update last switch and reset button states for next iteration
        self.last_switch = switch_button
        self.last_reset = reset_button
        self.last_save_ghost_point_button = save_ghost_point_button
        self.last_save_goal_point_button = save_goal_point_button



    # publish the switch mode status 
    def switch_robot_mode(self, status): 

        
        change_mode = Bool()
        change_mode.data = status        
        self.switchMode_pub.publish(change_mode)


    # publish the reset mode status 
    def reset_robot_odom(self, status): 


        # reset the state of the main machine states, for the initial one 
        reset_msg = Bool()
        reset_msg.data = status
        self.resetOdom_pub.publish(reset_msg)        

    

    def main(self): 




        current_time = self.get_clock().now()

        # Calculate linear and angular velocities from joystick input
        self.vel_linear = self.joy_vel_linear * (self.MAX_VEL_JOY_LINEAR / self.MAX_VALUE_CONTROLLER)
        self.vel_angular = self.joy_vel_angular * (self.MAX_VEL_JOY_ANGULAR / self.MAX_VALUE_CONTROLLER)  

        # Check if the joystick command timeout has been exceeded and the robot is in manual mode
        if (current_time - self.last_joy_command_time).nanoseconds > self.JOY_TIMEOUT  and self.manual_mode: 

            vel_linear = 0.0
            vel_angular = 0.0
        

        # Create Twist message for velocity command
        self.cmd_vel.linear.x = self.vel_linear
        self.cmd_vel.angular.z = -1 * self.vel_angular   # Angular velocity correction

        # Publish velocity command only if in manual mode
        if self.manual_mode: 
            
            self.vel_pub.publish(self.cmd_vel)


        if debug_mode or self.DEBUG:
             
            debug.joy_interface(self)

    def save_point(self, is_goal_point):

        pre_path = Path(get_package_share_directory("fred2_move_base")).parent.parent.parent.parent / 'src' / 'fred2_move_base' / 'data' 
        path = os.path.join(pre_path, "points_" + operation_time + ".txt")
        f = open(path, "a")
        
        f.write("{" + f"{self.position_x}, {self.position_y}, {self.position_z}" + "}" + f" is_goal: {is_goal_point}" + "\n")
        f.close()

if __name__ == '__main__': 
    
    rclpy.init(args= None, signal_handler_options= SignalHandlerOptions.NO)

    node = JoyInterfaceNode(
        node_name='joy_esp_interface',
        cli_args=['--debug'],
        namespace='move_base',
        enable_rosout=False
        )

    # separete theread for main function 
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(node.FREQUENCY)
    
    try: 
        while rclpy.ok(): 
            node.main()
            rate.sleep()
        
    except KeyboardInterrupt:

        node.get_logger().warn(' ------------------------------------ DEACTIVATING NODE --------------------------------------')
        pass
        
    rclpy.shutdown()
    thread.join()
