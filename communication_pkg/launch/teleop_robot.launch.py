from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nœud qui contrôle le PCA9685
        Node(
            package='actuation_pkg',
            node_executable='actuators',
            node_name='actuators',
            output='screen'
        ),
        # --- CapComNode en mode robot ---
        Node(
            package='communication_pkg',
            node_executable='capcom_node',     # script Python CapComNode
            node_name='capcom_node',
            output='screen',
            emulate_tty=True,   # <-- makes stdout unbuffered
            parameters=[{
                'com_port': '/dev/ttyTHS1',   # port Xbee sur le robot
                'com_baudrate': 57600,
                'remote_id': 'STATION',       # ID Xbee de la station
                'mode': 'robot'
            }]
        ),
    ])


