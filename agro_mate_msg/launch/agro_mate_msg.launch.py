from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agro_mate_msg',
            executable='agro_mate_msg',
            output='screen'),
    ])