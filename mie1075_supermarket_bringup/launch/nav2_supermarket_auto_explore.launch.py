from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    # --- Launch 参数 ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params = LaunchConfiguration('nav2_params')

    # --- 包路径 ---
    supermarket_pkg = get_package_share_directory('mie1075_supermarket_bringup')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    # 1) 启动 Gazebo + 超市 world + 你的小机器人
    supermarket_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                supermarket_pkg,
                'launch',
                'supermarket_tb3.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # 2) 启动 Nav2（内部起 slam_toolbox 做 SLAM）
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_pkg,
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'slam': 'True',   # ★ 告诉 nav2 用 SLAM 模式
            'map': ''         # ★ 不用现成地图
        }.items()
    )

    # 3) 自动设置初始位姿（代替你手动在 RViz 点 initial pose）
    auto_initialpose_node = Node(
        package='custom_explorer',
        executable='auto_initialpose',
        name='auto_initialpose',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'x': 0.0,     # 根据你机器人出生点需要可以再调
            'y': 0.0,
            'yaw': 0.0    # 单位：rad
        }]
    )

    # 4) 自动探索节点（发 NavigateToPose，让机器人自己跑）
    explorer_node = Node(
        package='custom_explorer',
        executable='explorer',
        name='explorer',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # --- 声明参数 ---
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (Gazebo clock)'
    )

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=os.path.join(
            supermarket_pkg,
            'nav2_config',
            'nav2_params.yaml'
        ),
        description='Nav2 params file'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_nav2_params,
        supermarket_launch,       # 世界 + 机器人
        nav2_launch,              # Nav2 + 内置 SLAM
        auto_initialpose_node,    # 自动 initial pose
        explorer_node,            # 自动探索
    ])
