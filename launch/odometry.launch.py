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

        TimerAction(period= 3.0, actions= [ 

            LogInfo(msg=' ######################### LAUNCHING LED MANAGER #################################### '), 
            odom_node
        ]), 
    ])
