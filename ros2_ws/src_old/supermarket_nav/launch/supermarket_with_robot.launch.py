from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 你的超市 world 文件路径
    world_path = os.path.expanduser("~/Documents/supermarket_model/supermarket.world")

    # 启动 Gazebo server + client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("gazebo_ros"),
            "/launch/gazebo.launch.py"
        ]),
        launch_arguments={"world": world_path}.items()
    )

    # TurtleBot3 模型文件路径
    TB3_MODEL = os.path.join(
        get_package_share_directory("turtlebot3_description"),
        "urdf",
        "turtlebot3_waffle_pi.urdf.xacro"
    )

    # robot_state_publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", TB3_MODEL])}]
    )

    # 在超市中生成机器人
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "tb3",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.01",
            "-file", TB3_MODEL
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_robot,
    ])
