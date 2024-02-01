import launch_ros
import launch_ros.descriptions

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Declare the launch argument
    declare_robot_localization_odom = DeclareLaunchArgument(
        '--use-robot-localization',
        default_value='true',  
        description='Use de odometry from robot localization'
    )

    
    safe_twist_node = launch_ros.actions.Node(

            package='fred2_move_base',
            executable='safe_twist.py',
            name='safe_twist',
            namespace='move_base', 
            arguments=[
                '--use-robot-localization',
                LaunchConfiguration('--use-robot-localization'),  
        ],
    )
    

    joy_interface_node = launch_ros.actions.Node(

        package='fred2_move_base',
        executable='joy_esp_interface.py',
        name='joy_esp_interface',
        namespace='move_base'
    )


    led_manager_node = launch_ros.actions.Node(

        package='fred2_move_base',
        executable='led_manager.py',
        name='led_manager',
        namespace='move_base'
    )


    odom_node = launch_ros.actions.Node(

        package='fred2_move_base',
        executable='ticks2odom.py',
        name='odometry',
        namespace='move_base'
    )


    return LaunchDescription([

        declare_robot_localization_odom,
        safe_twist_node, 
        joy_interface_node, 
        led_manager_node, 
        odom_node
    ])
