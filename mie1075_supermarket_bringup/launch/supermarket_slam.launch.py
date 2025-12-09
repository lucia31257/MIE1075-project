from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('mie1075_supermarket_bringup')
    slam_params = os.path.join(
        pkg_path,
        'MIE1075-project',
        'slam_config',
        'supermarket_slam.yaml'
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params],
            output='screen'
        )
    ])
