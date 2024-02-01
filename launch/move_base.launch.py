import launch_ros
import launch_ros.descriptions

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    safe_twist_node = launch_ros.actions.Node(

            package='fred2_move_base',
            executable='safe_twist.py',
            name='safe_twist',
            namespace='move_base'
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

        safe_twist_node, 
        joy_interface_node, 
        led_manager_node, 
        odom_node
    ])
