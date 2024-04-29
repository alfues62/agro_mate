from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoints_action',
            executable='action_client',
            output='screen'
        ),
    ])