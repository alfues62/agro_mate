from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='publicador_velocidad',
            executable='publicador_velocidad',
            output='screen'),
    ])