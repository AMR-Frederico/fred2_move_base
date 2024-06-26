    
from rclpy.node import Node, ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter, ParameterType



########################################################
# --------------- Set SAFE TWIST params 
########################################################

def get_safe_params(node: Node):

    # Declare parameters related to ultrasonic sensors, velocity limits, and debugging/testing
    node.declare_parameters(
        namespace='',
        parameters=[
            ('disable_ultrasonics', None, 
                ParameterDescriptor(
                    description='Toggle ultrasonic sensors on/off', 
                    type=ParameterType.PARAMETER_BOOL)),

            ('obstacle_detection_threshold', None, 
                ParameterDescriptor(
                    description='Stopping distance in cm when detecting an object with ultrasonic sensors', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('max_angular_speed', None, 
                ParameterDescriptor(
                    description='Maximum angular speed in degrees per second', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('max_linear_speed', None, 
                ParameterDescriptor(
                    description='Maximum linear speed in meters per second', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('motor_brake_factor', None, 
                ParameterDescriptor(
                    description='Brake factor for motors; negative values indicate braking force', 
                    type=ParameterType.PARAMETER_INTEGER)),
            
            ('joy_low_battery_threshold', None, 
                ParameterDescriptor(
                    description='Define a threshold for low joystick battery', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('cmd_vel_timeout', None, 
                ParameterDescriptor(
                    description='Timeout duration (in nanoseconds) to reset the cmd_vel status if no message is received within this time.', 
                    type=ParameterType.PARAMETER_INTEGER)),
            
            ('joy_connection_timeout', None, 
                ParameterDescriptor(
                    description='Timeout duration (in nanoseconds) to reset the joy_connected status if no message is received within this time', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('frequency', None, 
                ParameterDescriptor(
                    description='Node frequency', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('debug', None, 
                ParameterDescriptor(
                    description='Enable debug prints for troubleshooting', 
                    type=ParameterType.PARAMETER_BOOL))
            
        ]
    )

    node.OBSTACLE_DETECTION_THRESHOLD = node.get_parameter('obstacle_detection_threshold').value 
    node.DISABLE_ULTRASONICS = node.get_parameter('disable_ultrasonics').value

    node.MOTOR_BRAKE_FACTOR = node.get_parameter('motor_brake_factor').value
    node.MAX_LINEAR_SPEED = node.get_parameter('max_linear_speed').value
    node.MAX_ANGULAR_SPEED = node.get_parameter('max_angular_speed').value
    
    node.JOY_LOW_BATTERY_THRESHOLD = node.get_parameter('joy_low_battery_threshold').value

    node.JOY_TIMEOUT = node.get_parameter('joy_connection_timeout').value
    node.VEL_TIMEOUT = node.get_parameter('cmd_vel_timeout').value

    node.DEBUG = node.get_parameter('debug').value
    node.FREQUENCY = node.get_parameter('frequency').value






########################################################
# --------------- Update SAFE TWIST params 
########################################################

# updates the parameters when they are changed by the command line
def safe_params_callback(node, params):
    
    for param in params:
        node.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")


    if param.name == 'obstacle_detection_threshold':
        node.OBSTACLE_DETECTION_THRESHOLD = param.value


    if param.name == 'motor_brake_factor':
        node.MOTOR_BRAKE_FACTOR = param.value


    if param.name == 'max_linear_speed': 
        node.MAX_LINEAR_SPEED = param.value


    if param.name == 'max_angular_speed': 
        node.MAX_ANGULAR_SPEED = param.value

    
    if param.name == 'joy_low_battery_threshold': 
        node.JOY_LOW_BATTERY_THRESHOLD = param.value


    if param.name =='cmd_vel_timeout': 
        node.VEL_TIMEOUT = param.value


    if param.name == 'joy_timeout': 
        node.JOY_TIMEOUT = param.value 


    if param.name == 'disable_ultrasonics': 
        node.DISABLE_ULTRASONICS = param.value
    

    if param.name == 'frequency': 
        node.FREQUENCY = param.value 


    if param.name == 'debug': 
        node.DEBUG = param.value


    return SetParametersResult(successful=True)



