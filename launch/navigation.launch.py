# Copyright 2026 longlong
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch Gazebo and Nav2 localization with a previously saved map."""
    package_name = 'my_underscore_bot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')
    spawn_yaw = LaunchConfiguration('yaw')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    start_rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package_name),
                'launch',
                'launch_sim.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'x': spawn_x,
            'y': spawn_y,
            'z': spawn_z,
            'yaw': spawn_yaw,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ),
        launch_arguments={
            'slam': 'False',
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
    )

    delayed_nav2 = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='仿真已就绪，正在启动 Nav2 定位与导航...'),
            nav2,
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='my_world.sdf',
            description='Gazebo world file from the worlds directory',
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Robot spawn X coordinate in metres',
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Robot spawn Y coordinate in metres',
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.0',
            description='Robot spawn Z coordinate in metres',
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Robot spawn yaw in radians',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare(package_name),
                'maps',
                'my_map.yaml',
            ]),
            description='Full path to the saved map YAML file',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare(package_name),
                'config',
                'nav2_params.yaml',
            ]),
            description='Full path to the Nav2 parameter file',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate the Nav2 lifecycle nodes',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with the project navigation view',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare(package_name),
                'config',
                'view_bot_rviz.rviz',
            ]),
            description='Full path to the RViz configuration file',
        ),
        LogInfo(msg='正在启动 Gazebo 保存地图导航模式...'),
        simulation,
        delayed_nav2,
        rviz,
    ])
