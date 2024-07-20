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

    led_manager_node = launch_ros.actions.Node(
        package='fred2_move_base',
        executable='led_manager.py',
        name='led_manager',
        namespace='move_base',
        parameters=[config],
    )


    return LaunchDescription([

        TimerAction(period= 3.0, actions= [

            LogInfo(msg=' ######################### LAUNCHING ODOMETRY #################################### '), 
            led_manager_node
        ])
    ])
