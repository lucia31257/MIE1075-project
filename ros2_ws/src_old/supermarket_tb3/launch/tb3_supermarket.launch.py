from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # ======== 1. Your supermarket world path ========
    world_path = '/home/lwl/supermarket.world'

    # ======== 2. Launch Gazebo Classic with this world ========
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_path}.items()
    )

    # ======== 3. TurtleBot3 model path ========
    tb3_sdf_path = '/home/lwl/.gazebo/models/turtlebot3_waffle_pi/model.sdf'

    # ======== 4. Spawn robot ========
    spawn_tb3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3',
            '-file', tb3_sdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',
        ],
        output='screen'
    )

    # ======== 5. Delay spawn so world loads first ========
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_tb3]
    )

    return LaunchDescription([
        gazebo_launch,
        delayed_spawn
    ])
