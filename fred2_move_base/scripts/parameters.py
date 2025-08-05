    
from rclpy.node import Node, ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter, ParameterType

from rcl_interfaces.srv import GetParameters




########################################################
# --------------- Set SAFE TWIST params 
########################################################

def safe_twist(node: Node):

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


########################################################
# --------------- ODOMETRY params 
########################################################

# Updates the parameters when they are changed by the command line
def odom_parameters_callback(node: Node, params):
    
    for param in params:
        node.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")


    if param.name == 'wheels_track':
        node.WHEELS_TRACK = param.value


    if param.name == 'wheels_radius':
        node.WHEELS_RADIUS = param.value


    if param.name == 'ticks_per_revolution': 
        node.TICKS_PER_TURN = param.value


    if param.name == 'base_link_offset': 
        node.BASE_LINK_OFFSET = param.value
    
    
    if param.name == 'debug': 
        node.DEBUG = param.value
    

    if param.name == 'frequency': 
        node.FREQUENCY = param.value 

    
    if param.name == 'publish_odom_tf': 
        node.PUBLISH_ODOM_TF = param.value 


    return SetParametersResult(successful=True)


def odometry_config(node: Node):

    # Declare parameters related to robot dimensions, encoder configuration, and debugging/testing
    node.declare_parameters(
        namespace='',
        parameters=[
            ('base_link_offset', 0.08, 
                ParameterDescriptor(
                    description='Offset of the base link from the ground in meters', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('wheels_radius', 0.075, 
                ParameterDescriptor(
                    description='Radius of the wheels in meters', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('wheels_track', 0.38, 
                ParameterDescriptor(
                    description='Distance between the centers of the left and right wheels in meters', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('ticks_per_revolution', 1024, 
                ParameterDescriptor(
                    description='Number of encoder ticks per revolution of the wheel', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('frequency', 6, 
                ParameterDescriptor(
                    description='Node frequency', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('publish_odom_tf', True, 
                ParameterDescriptor(
                    description='Allows the node to publish odom tf ', 
                    type=ParameterType.PARAMETER_BOOL)),

            ('debug', True, 
                ParameterDescriptor(
                    description='Enable debug prints for troubleshooting', 
                    type=ParameterType.PARAMETER_BOOL)),
        ]
    )

    node.WHEELS_TRACK = node.get_parameter('wheels_track').value 
    node.WHEELS_RADIUS = node.get_parameter('wheels_radius').value
    node.TICKS_PER_TURN = node.get_parameter('ticks_per_revolution').value
    node.BASE_LINK_OFFSET = node.get_parameter('base_link_offset').value

    node.FREQUENCY = node.get_parameter('frequency').value 

    node.PUBLISH_ODOM_TF = node.get_parameter('publish_odom_tf').value

    node.DEBUG = node.get_parameter('debug').value



########################################################
# --------------- LED MANAGER params 
########################################################


    # Declare parameters related to LED colors, goal indices, and debugging/testing
def led_config(node: Node):

    node.declare_parameters(
        namespace='',
        parameters=[
            ('black', None, 
                ParameterDescriptor(
                    description='Color index for black', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('blue', None, 
                ParameterDescriptor(
                    description='Color index for blue', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('cyan', None, 
                ParameterDescriptor(
                    description='Color index for cyan', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('green', None, 
                ParameterDescriptor(
                    description='Color index for green', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('light_green', None, 
                ParameterDescriptor(
                    description='Color index for light green', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('orange', None, 
                ParameterDescriptor(
                    description='Color index for orange', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('pink', None, 
                ParameterDescriptor(
                    description='Color index for pink', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('purple', None, 
                ParameterDescriptor(
                    description='Color index for purple', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('red', None, 
                ParameterDescriptor(
                    description='Color index for red', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('yellow', None, 
                ParameterDescriptor(
                    description='Color index for yellow', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('white', None, 
                ParameterDescriptor(
                    description='Color index for white', 
                    type=ParameterType.PARAMETER_INTEGER)),
            
            ('frequency', None, 
                ParameterDescriptor(
                    description='Node frequency', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('debug', None, 
                ParameterDescriptor(
                    description='Enable debug prints for troubleshooting', 
                    type=ParameterType.PARAMETER_BOOL)),

            ('use_global_param', None, 
                ParameterDescriptor(
                    description='Enable unit testing mode', 
                    type=ParameterType.PARAMETER_BOOL))
        ]
    )
    
    # Get index colors
    node.WHITE = node.get_parameter('white').value
    node.BLUE = node.get_parameter('blue').value
    node.YELLOW = node.get_parameter('yellow').value
    node.PINK = node.get_parameter('pink').value
    node.ORANGE = node.get_parameter('orange').value 
    node.RED = node.get_parameter('red').value
    node.GREEN = node.get_parameter('green').value 
    node.BLACK = node.get_parameter('black').value 
    node.CYAN = node.get_parameter('cyan').value
    node.PURPLE = node.get_parameter('purple').value
    node.LIGHT_GREEN = node.get_parameter('light_green').value    
    
    node.GLOBAL_PARAMS = node.get_parameter('use_global_param').value
    node.DEBUG = node.get_parameter('debug').value
    node.FREQUENCY = node.get_parameter('frequency').value 

    if node.GLOBAL_PARAMS:

        global_params(node)




def led_params_callback(self, params):
    
    for param in params:
        self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")


    if param.name == 'white':
        self.WHITE = param.value


    if param.name == 'blue':
        self.BLUE = param.value


    if param.name == 'yellow': 
        self.YELLOW = param.value


    if param.name == 'pink': 
        self.PINK = param.value
    

    if param.name == 'orange': 
        self.ORANGE = param.value


    if param.name == 'red': 
        self.RED = param.value


    if param.name == 'green': 
        self.GREEN = param.value


    if param.name == 'black': 
        self.BLACK = param.value


    if param.name == 'cyan': 
        self.CYAN = param.value


    if param.name == 'purple': 
        self.PURPLE = param.value    


    if param.name == 'light_green': 
        self.LIGHT_GREEN = param.value        

    
    if param.name == 'debug': 
        self.DEBUG = param.value

    
    if param.name == 'use_global_param': 
        self.GLOBAL_PARAMS = param.value 
    
    if param.name == 'frequency': 
        self.FREQUENCY = param.value 



    return SetParametersResult(successful=True)
        
        

########################################################
# --------------- JOY INTERFACE params 
########################################################


def joy_config(node: Node):


    # Declare parameters for joystick and velocity limits, as well as debug settings
    node.declare_parameters(
        namespace='',
        parameters=[
            ('drift_analog_tolerance', None, 
                ParameterDescriptor(
                    description='Tolerance for analog drift in joystick', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('max_value_controller', None, 
                ParameterDescriptor(
                    description='Maximum value for joystick controller', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('max_vel_joy_angular', None, 
                ParameterDescriptor(
                    description='Maximum angular velocity from joystick', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('max_vel_joy_linear', None, 
                ParameterDescriptor(
                    description='Maximum linear velocity from joystick', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('debug', None, 
                ParameterDescriptor(
                    description='Enable debug prints for troubleshooting',
                    type=ParameterType.PARAMETER_BOOL)),
                    
            ('use_global_param', None, 
                ParameterDescriptor(
                    description='Enable unit testing mode', 
                    type=ParameterType.PARAMETER_BOOL)),
            
            ('frequency', None, 
                ParameterDescriptor(
                    description='Node frequency', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('joy_cmd_timeout', None, 
                ParameterDescriptor(
                    description='Timeout duration (in nanoseconds) to reset the joy_cmd status if no message is received within this time.', 
                    type=ParameterType.PARAMETER_INTEGER)),
        ]
    )

    node.get_logger().info('All parameters successfully declared')


    node.MAX_VEL_JOY_LINEAR = node.get_parameter('max_vel_joy_linear').value
    node.MAX_VEL_JOY_ANGULAR = node.get_parameter('max_vel_joy_angular').value

    node.MAX_VALUE_CONTROLLER = node.get_parameter('max_value_controller').value
    node.DRIFT_ANALOG_TOLERANCE = node.get_parameter('drift_analog_tolerance').value
    node.JOY_TIMEOUT = node.get_parameter('joy_cmd_timeout').value 

    node.FREQUENCY = node.get_parameter('frequency').value
    node.GLOBAL_PARAMS = node.get_parameter('use_global_param').value
    node.DEBUG = node.get_parameter('debug').value


    # Allows it to run without the machine states 
    if node.GLOBAL_PARAMS: 
        
        global_params(node)


# updates the parameters when they are changed by the command line
def joy_params_callback(self, params):
    
    for param in params:
        self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")


    if param.name == 'max_vel_joy_linear':
        self.MAX_VEL_JOY_LINEAR = param.value


    if param.name == 'max_vel_joy_angular':
        self.MAX_VEL_JOY_ANGULAR = param.value


    if param.name == 'max_value_controller': 
        self.MAX_VALUE_CONTROLLER = param.value


    if param.name == 'drift_analog_tolerance': 
        self.DRIFT_ANALOG_TOLERANCE = param.value


    if param.name == 'debug': 
        self.DEBUG = param.value 


    if param.name == 'unit_test': 
        self.UNIT_TEST = param.value 
    

    if param.name == 'frequency': 
        self.FREQUENCY = param.value 


    if param.name == 'joy_cmd_timeout': 
        self.JOY_TIMEOUT = param.value


    return SetParametersResult(successful=True)



########################################################
# --------------- GLOBAL PARAMETERS params 
########################################################


def global_params(node: Node): 

    # Get global params 
    node.client = node.create_client(GetParameters, '/main_robot/operation_modes/get_parameters')
    node.client.wait_for_service()

    request = GetParameters.Request()
    request.names = ['init', 'manual', 'autonomous', 'emergency']

    future = node.client.call_async(request)
    future.add_done_callback(lambda future: callback_global_param(node, future))


    node.get_logger().info('Global params are deactivated')  
        


# get the global values from the machine states params 
def callback_global_param(node: Node, future):


    try:

        result = future.result()

        node.ROBOT_INIT = result.values[0].integer_value
        node.ROBOT_MANUAL = result.values[1].integer_value
        node.ROBOT_AUTONOMOUS = result.values[2].integer_value
        node.ROBOT_EMERGENCY = result.values[3].integer_value

        node.get_logger().info(f"Got global param ROBOT_EMERGENCY: {node.ROBOT_EMERGENCY}\n")
        node.get_logger().info(f"Got global param ROBOT_INIT -> {node.ROBOT_INIT}")
        node.get_logger().info(f"Got global param ROBOT_MANUAL -> {node.ROBOT_MANUAL}")
        node.get_logger().info(f"Got global param ROBOT_AUTONOMOUS -> {node.ROBOT_AUTONOMOUS}")



    except Exception as e:

        node.get_logger().warn("Service call failed %r" % (e,))

