from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

def general_configuration():

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,  # Set the reliability policy to RELIABLE, ensuring reliable message delivery
        durability= QoSDurabilityPolicy.VOLATILE,   # Set the durability policy to VOLATILE, indicating messages are not stored persistently
        history=QoSHistoryPolicy.KEEP_LAST,         # Set the history policy to KEEP_LAST, storing a limited number of past messages
        depth=10,                                   # Set the depth of the history buffer to 10, specifying the number of stored past messages
        liveliness=QoSLivelinessPolicy.AUTOMATIC    # Set the liveliness policy to AUTOMATIC, allowing automatic management of liveliness
        
    )

    return qos_profile



def imu_configuration(): 
        
    # quality protocol -> the node must not lose any message 
    qos_imu_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,   # Set the reliability policy to BEST EFFORT, ensuring best efford message delivery
        durability= QoSDurabilityPolicy.VOLATILE,       # Set the durability policy to VOLATILE, indicating messages are not stored persistently
        history=QoSHistoryPolicy.KEEP_LAST,             # Set the history policy to KEEP_LAST, storing a limited number of past messages
        depth=10,                                       # Set the depth of the history buffer to 10, specifying the number of stored past messages
        liveliness=QoSLivelinessPolicy.AUTOMATIC        # Set the liveliness policy to AUTOMATIC, allowing automatic management of liveliness
        
    )

    return qos_imu_profile
