from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        # Autonomous explorer node
        Node(
            package='custom_explorer',
            executable='explorer',
            name='explorer_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),

        # Optional initial pose setter
        Node(
            package='custom_explorer',
            executable='auto_initialpose',
            name='initialpose_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),
    ])
