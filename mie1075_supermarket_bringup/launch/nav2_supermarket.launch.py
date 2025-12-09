import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = True

    pkg_dir = get_package_share_directory('mie1075_supermarket_bringup')
    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(bringup_dir, 'launch', 'bringup_launch.py')

    slam_params = os.path.join(pkg_dir, 'slam_config', 'supermarket_slam.yaml')
    nav2_params = os.path.join(pkg_dir, 'nav2_config', 'nav2_params.yaml')

    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'slam_params_file': slam_params},
        ]
    )

    # Nav2 bringup
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'map': '__dummy__',
            'params_file': nav2_params
        }.items()
    )

    return LaunchDescription([
        slam_node,
        nav2,
    ])
