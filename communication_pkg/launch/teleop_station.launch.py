from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nœud ROS2 joy pour la manette
        Node(
            package='joy',
            executable='joy_node',  # <- ancien paramètre pour Eloquent
            name='joy_node',
            output='screen',
            parameters=[{'deadzone': 0.1}]
        ),
        # --- Nœud Joystick (actuation_pkg) ---
        Node(
            package='actuation_pkg',      # package où se trouve ton joystick
            executable='actuation_joystick',   # nom du script Python ou binaire
            name='joystick_node',
            output='screen'
        ),
        # --- Nœud Joystick (actuation_pkg) ---
        Node(
            package='sensor_pkg',      # package où se trouve ton joystick
            executable='video_node',   # nom du script Python ou binaire
            name='camera_teleop',
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
        ),
        # --- rqt_image_view pour visualiser le flux caméra ---
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view_node',
            output='screen',
            arguments=['/camera/image_raw']  # topic à visualiser
        )
    ])

