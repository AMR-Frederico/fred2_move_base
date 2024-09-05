from rclpy.node import Node



def safe_twist(node: Node): 

    node.get_logger().info( 
        
        f"safety: {node.robot_safety} | abort: {node.user_abort_command} | collision: {node.stop_by_obstacle} | Joy -> status:{node.joy_connected}, battery: {node.joy_battery} | ultra -> disabled: {node.DISABLE_ULTRASONICS} | L: {node.left_ultrasonic_distance}, R: {node.right_ultrasonic_distance}, B: {node.back_ultrasonic_distance} | robot_vel -> x: {node.robot_vel.linear.x}, z: {node.robot_vel.angular.z} | braking: {node.deceleration_factor} | cmd_vel -> x: {node.cmd_vel.linear.x}, z: {node.cmd_vel.angular.z}, | safe/cmd_vel -> x: {node.cmd_vel_safe.linear.x}, z: {node.cmd_vel_safe.angular.z}"
    )



def odometry(node: Node): 

    node.get_logger().info(
        
        f'Position -> x: {node.x_pos} | y: {node.y_pos} | theta: {node.theta} | Velocity -> x: {node.linear_vel_x} | y: {node.linear_vel_y} | theta: {node.angular_vel_theta} | Ticks -> left: {node.left_wheels_ticks} | right: {node.right_wheels_ticks}'
    )



def joy_interface(node: Node): 
                
    node.get_logger().info(
          
        f'Velocity -> linear:{node.vel_linear} | angular:{node.vel_angular} | Reset odometry -> {node.reset_odom} | Switch mode -> {node.switch_mode} | Manual mode -> {node.manual_mode}'
    )


def led_manager(node: Node): 

    node.get_logger().info(

        f"Color: {node.led_color.data} | Debug color: {node.led_debug.data} | Collision alert: {node.collision_detected} | Stop command: {node.user_stop_command} | Joy connected: {node.joy_connected} | Ultrasonics disabled: {node.ultrasonic_disabled} | Robot state: {node.robot_state}"
    )