from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nœud ROS2 joy pour la manette
        Node(
            package='joy',
            node_executable='joy_node',  # <- ancien paramètre pour Eloquent
            node_name='joy_node',
            output='screen',
            parameters=[{'deadzone': 0.1}]
        ),

        # Nœud qui contrôle le PCA9685
        Node(
            package='actuation_pkg',
            node_executable='actuators',
            node_name='actuators',
            output='screen'
        ),

        # Nœud qui convertit les entrées manette en commandes pour les servos
        Node(
            package='actuation_pkg',
            node_executable='actuation_joystick',
            node_name='actuation_joystick',
            output='screen'
        ),
    ])

