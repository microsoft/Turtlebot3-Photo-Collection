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

    turtlebot3_bringup = get_package_share_directory('turtlebot3_bringup')
    
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('photo_collector'),
            'config',
            'custom_vision.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot3_bringup, '/launch/robot.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path to param file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='photo_collector',
            executable='main',
            name='main',
            output='screen',
            parameters=[param_dir],
            arguments=["False"]),
    ])