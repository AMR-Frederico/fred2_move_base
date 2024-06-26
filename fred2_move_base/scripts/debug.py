from rclpy.node import Node

def safe_debug(node: Node): 

    node.get_logger().warn(f'Robot safety -> {node.robot_safety}')
    node.get_logger().info(f'Emergency mode -> user comand abort: {node.user_abort_command} | collision detected: {node.stop_by_obstacle}')
    node.get_logger().info(f'Emergency mode -> joy connected: {node.joy_connected} | ultrasonics disabled: {node.DISABLE_ULTRASONICS}')
    node.get_logger().info(f'Ultrasonics -> left: {node.left_ultrasonic_distance} | right: {node.right_ultrasonic_distance} | Back: {node.back_ultrasonic_distance}')
    node.get_logger().info(f'Robot velocity -> linear: {node.robot_vel.linear.x} | angular: {node.robot_vel.angular.z}')
    node.get_logger().info(f'Velocity command -> linear: {node.cmd_vel.linear.x} | angular: {node.cmd_vel.angular.z} | braking_factor: {node.deceleration_factor}')
    node.get_logger().info(f'Safe velocity command -> linear: {node.cmd_vel_safe.linear.x} | angular: {node.cmd_vel_safe.angular.z}\n')
