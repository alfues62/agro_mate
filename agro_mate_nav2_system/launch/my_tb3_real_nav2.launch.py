import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    nav2_yaml = os.path.join(get_package_share_directory('agro_mate_nav2_system'), 'config', 'my_nav2_params_real.yaml')
    map_file = os.path.join(get_package_share_directory('agro_mate_nav2_system'), 'config', 'my_map_REAL.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('agro_mate_nav2_system'), 'config', 'agribot_v2.rviz')
   # urdf = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_burger.urdf')
   # world = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_worlds/burger.model')


    return LaunchDescription([
        Node(
            package = 'nav2_map_server',
            executable = 'map_server',
            name = 'map_server',
            output = 'screen',
            parameters=[{'use_sim_time': False}, {'yaml_filename':map_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),


        Node(
            package = 'nav2_planner',
            executable = 'planner_server',
            name = 'planner_server',
            output = 'screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': False}]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': False}]
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': False}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': False}],
            output='screen'
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': False}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names':['map_server', 'amcl', 'planner_server', 'controller_server', 'recoveries_server', 'bt_navigator', 'waypoint_follower']}]
        )
    ])