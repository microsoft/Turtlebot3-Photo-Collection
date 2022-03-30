import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    turtlebot3_cartographer = get_package_share_directory('turtlebot3_cartographer')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav2_bringup, '/launch/navigation_launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time}.items(),
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot3_cartographer, '/launch/cartographer.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        Node(
            package='photo_collector',
            executable='poi_broadcaster',
            name='poi_broadcaster',
            output='screen',
            parameters=[],
            arguments=[]),
    ])
