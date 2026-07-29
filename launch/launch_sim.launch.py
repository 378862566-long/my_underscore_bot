import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ============================================================
    # 包名和路径
    # ============================================================
    package_name = 'my_underscore_bot'
    pkg_share = get_package_share_directory(package_name)

    # 编译 xacro → URDF（包含 ros2_control 标签和 GazeboSimROS2ControlPlugin）
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # 仿真时间标志
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ============================================================
    # 1. robot_state_publisher — 发布 TF 变换
    # ============================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    # ============================================================
    # 2. Gazebo 仿真（空世界，自动开始运行）
    # ============================================================

    world_file = os.path.join(pkg_share, 'worlds', 'my_world.sdf')


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # ============================================================
    # 3. 在 Gazebo 中生成机器人
    #    URDF 包含 <plugin name="GazeboSimROS2ControlPlugin">,
    #    该插件自带 controller_manager, 不需要单独启动 ros2_control_node
    #    -z 0.05 让轮子底部刚好接触地面
    # ============================================================
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_underscore_bot',
                   '-z', '0.05'],
        output='screen'
    )

    # ============================================================
    # 4. 控制器加载器 — 连接到插件内置的 controller_manager
    #    延迟启动等 spawn 和 GazeboSimROS2ControlPlugin 初始化完成
    # ============================================================
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '-c', '/controller_manager'],
        output='screen',
    )

    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '-c', '/controller_manager'],
        output='screen',
    )

    # 将键盘默认发布的 Twist 转为带仿真时间戳的 TwistStamped。
    cmd_vel_relay = Node(
        package=package_name,
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 延迟 6 秒加载控制器（确保 Gazebo 完全启动 + 插件初始化完毕）
    delayed_spawners = TimerAction(
        period=6.0,
        actions=[joint_broad_spawner, diff_cont_spawner],
    )

    # ============================================================
    # 启动描述
    # ============================================================
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用仿真时间'),
        LogInfo(msg='正在启动仿真环境...'),
        robot_state_publisher_node,     # 1. TF 发布
        gazebo,                         # 2. Gazebo 仿真
        spawn_entity,                   # 3. 生成机器人（含 controller_manager）
        delayed_spawners,               # 4. 延迟加载 diff_drive 等控制器
        cmd_vel_relay,                  # 5. /cmd_vel 兼容桥接
    ])
