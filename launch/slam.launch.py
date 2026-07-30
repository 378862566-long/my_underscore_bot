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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the Gazebo simulation and online asynchronous SLAM."""
    package_name = 'my_underscore_bot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')
    spawn_yaw = LaunchConfiguration('yaw')
    slam_params_file = LaunchConfiguration('slam_params_file')
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

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
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
            'slam_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare(package_name),
                'config',
                'mapper_params_online_async.yaml',
            ]),
            description='Full path to the SLAM Toolbox parameter file',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz with the project mapping configuration',
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
        LogInfo(msg='正在启动 Gazebo 在线建图...'),
        simulation,
        slam_toolbox,
        rviz,
    ])
