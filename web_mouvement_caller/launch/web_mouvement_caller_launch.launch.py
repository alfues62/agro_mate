from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='web_mouvement_caller',
            executable='web_movement_server',
            output='screen'
        ),
    ])