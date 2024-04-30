from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agro_mate_point_action',
            executable='action_client',
            output='screen'
        ),
    ])