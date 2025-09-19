from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- Nœud Joystick (actuation_pkg) ---
        Node(
            package='actuation_pkg',      # package où se trouve ton joystick
            executable='actuation_joystick',   # nom du script Python ou binaire
            name='joystick_node',
            output='screen'
        ),
        # --- CapComNode en mode teleop ---
        Node(
            package='communication_pkg',  # ton package CapComNode
            executable='capcom_node',    # script Python CapComNode
            name='capcom_teleop',
            output='screen',
            parameters=[
                {'com_port': '/dev/ttyUSB0'},   # port Xbee sur la station
                {'com_baudrate': 57600},
                {'remote_id': 'ULTIMA'},         # ID Xbee du robot
                {'mode': 'teleop'}
            ]
        )
    ])

