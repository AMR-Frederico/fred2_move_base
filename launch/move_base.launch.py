import os
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
    

    config = os.path.join(
        get_package_share_directory('fred2_move_base'),
        'config',
        'move_base_params.yaml'
        )

    # Declare the launch argument
    declare_publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='false',  
        description='Whether to publish TF or not'
    )


    safe_twist_node = launch_ros.actions.Node(
        package='fred2_move_base',
        executable='safe_twist.py',
        name='safe_twist',
        namespace='move_base',
        parameters=[config],
    )


    joy_interface_node = launch_ros.actions.Node(
        package='fred2_move_base',
        executable='joy_esp_interface.py',
        name='joy_esp_interface',
        namespace='move_base',
        parameters=[config],
    )


    led_manager_node = launch_ros.actions.Node(
        package='fred2_move_base',
        executable='led_manager.py',
        name='led_manager',
        namespace='move_base',
        parameters=[config],
    )


    odom_node = launch_ros.actions.Node(
        package='fred2_move_base',
        executable='ticks2odom.py',
        name='odometry',
        namespace='move_base',
        parameters=[config],
        arguments=[
            '--publish-tf',
            LaunchConfiguration('publish_tf'),  # Pass the value from the launch argument
        ],
    )
    

    return LaunchDescription([

        declare_publish_tf_arg,

        TimerAction(period= 5.0, actions= [
            
            LogInfo(msg=' ######################### LAUNCHING SAFE TWIST #################################### '), 
            safe_twist_node
        ]), 

        TimerAction(period= 3.0, actions=[

            LogInfo(msg=' ######################### LAUNCHING JOY ESP INTERFACE #################################### '), 
            joy_interface_node
        ]), 

        TimerAction(period= 3.0, actions= [ 

            LogInfo(msg=' ######################### LAUNCHING LED MANAGER #################################### '), 
            odom_node
        ]), 

        TimerAction(period= 3.0, actions= [

            LogInfo(msg=' ######################### LAUNCHING ODOMETRY #################################### '), 
            led_manager_node
        ])
    ])
