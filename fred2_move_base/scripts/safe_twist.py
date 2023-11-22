#!/usr/bin/env python3
import rclpy
import threading

from typing import List, Optional

from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.context import Context 
from rclpy.node import Node

from std_msgs.msg import Bool

class SafeTwistNode(Node):
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
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=1
        )
        
        self.create_subscription(Bool, 'your_topic', self.callback, qos_profile)

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main():
    print('oi')

if __name__ == '__main__':
    # Create a custom context for single thread and real-time execution
    rclpy.init()

    safe_context = rclpy.Context()
    safe_context.init()
    safe_context.use_real_time = True
    

    node = SafeTwistNode(
        node_name='safe_twist',
        context=safe_context,
        cli_args=['--debug', '--disable_ultrasonics'],
        namespace='move_base',
        enable_rosout=False
    )

    # Make the execution in real time 
    executor = SingleThreadedExecutor(context=safe_context)
    executor.add_node(node)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(10)

    try: 
        while rclpy.ok(): 
            main()
            rate.sleep()
        
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
