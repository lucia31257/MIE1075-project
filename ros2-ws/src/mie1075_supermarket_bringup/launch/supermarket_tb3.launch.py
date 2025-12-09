import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # ======================================================
    # 1. 声明参数
    # ======================================================
    # 仿真时间参数
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 初始位置参数 (默认回到了原点 0.0)
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0', description='Initial x position')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0', description='Initial y position')
    yaw_pose_arg = DeclareLaunchArgument('yaw_pose', default_value='0.0', description='Initial yaw orientation')

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    # ======================================================
    # 2. 路径配置
    # ======================================================
    # 超市地图路径
    world_path = os.path.join(
        os.path.expanduser('~'),
        'MIE1075-project',
        'supermarket.world'
    )

    # 机器人 SDF 模型路径 (用于 Gazebo 显示)
    sdf_path = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws',
        'src',
        'mie1075_supermarket_bringup',
        'models',
        'turtlebot3_custom',
        'model.sdf'
    )
    
    # 打印一下路径，确保系统能找到
    print(f"Loading Custom SDF: {sdf_path}")

    # ROS URDF 路径 (用于 TF 坐标树)
    urdf_file_name = 'turtlebot3_waffle_pi.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name
    )

    # ======================================================
    # 3. 解析 URDF (关键修复)
    # ======================================================
    print("URDF Path:", urdf_path) 
    # 使用 xacro 解析，解决 ${namespace} 报错问题
    robot_desc = Command(['xacro ', urdf_path])

    # ======================================================
    # 4. 节点定义
    # ======================================================
    
    # A. 启动 Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # B. Robot State Publisher (发布机器人身体坐标)
    # 必须传入解析后的 robot_desc 和 use_sim_time
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time, 
            'robot_description': robot_desc
        }],
    )

    # C. Joint State Publisher (发布关节状态)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # D. Spawn Entity (在 Gazebo 中生成机器人)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3',
            '-file', sdf_path,
            '-x', x_pose,   # 使用上面声明的参数 (默认 0.0)
            '-y', y_pose,   # 使用上面声明的参数 (默认 0.0)
            '-z', '0.05',   # 稍微抬高一点，防止卡在地里
            '-Y', yaw_pose, # 朝向 (默认 0.0)
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # E. Static TF (静态坐标变换)
    static_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_laser_tf',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'base_footprint',
            'base_scan'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] 
    )

    return LaunchDescription([
        sim_time_arg,
        x_pose_arg,    # 加入参数
        y_pose_arg,    # 加入参数
        yaw_pose_arg,  # 加入参数
        gazebo_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        static_laser_tf,
        # 延时 8 秒再生成机器人，防止 Gazebo 还没加载完导致服务超时报错
        TimerAction(period=8.0, actions=[spawn_entity]),
    ])