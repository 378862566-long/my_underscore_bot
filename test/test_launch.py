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

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'launch_sim.launch.py'
SLAM_LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'slam.launch.py'


def load_launch_module():
    """Load the simulation launch file as a Python module."""
    module_spec = importlib.util.spec_from_file_location(
        'launch_sim',
        LAUNCH_FILE,
    )
    launch_module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(launch_module)
    return launch_module


def generate_launch_description():
    """Generate a fresh launch description for each test."""
    launch_module = load_launch_module()
    return launch_module.generate_launch_description()


def generate_slam_launch_description():
    """Load and generate the SLAM launch description."""
    module_spec = importlib.util.spec_from_file_location(
        'slam_launch',
        SLAM_LAUNCH_FILE,
    )
    launch_module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(launch_module)
    return launch_module.generate_launch_description()


def test_launch_file_generates_launch_description():
    """Verify that the launch file can be loaded and evaluated."""
    launch_description = generate_launch_description()

    assert isinstance(launch_description, LaunchDescription)


def test_required_launch_arguments_are_declared():
    """Verify that all configurable simulation arguments are available."""
    launch_description = generate_launch_description()
    argument_names = {
        action.name
        for action in launch_description.entities
        if isinstance(action, DeclareLaunchArgument)
    }

    expected_arguments = {
        'use_sim_time',
        'world',
        'x',
        'y',
        'z',
        'yaw',
    }

    assert expected_arguments.issubset(argument_names)


def test_launch_description_contains_runtime_actions():
    """Verify that declarations are followed by simulation actions."""
    launch_description = generate_launch_description()
    runtime_actions = [
        action
        for action in launch_description.entities
        if not isinstance(action, DeclareLaunchArgument)
    ]

    assert runtime_actions


def test_slam_launch_arguments_are_declared():
    """Verify simulation and SLAM configuration arguments."""
    launch_description = generate_slam_launch_description()
    argument_names = {
        action.name
        for action in launch_description.entities
        if isinstance(action, DeclareLaunchArgument)
    }

    expected_arguments = {
        'use_sim_time',
        'world',
        'x',
        'y',
        'z',
        'yaw',
        'slam_params_file',
        'rviz',
        'rviz_config',
    }

    assert expected_arguments.issubset(argument_names)


def test_slam_launch_contains_async_slam_node():
    """Verify that the asynchronous SLAM Toolbox node is launched."""
    launch_description = generate_slam_launch_description()
    slam_nodes = [
        action
        for action in launch_description.entities
        if (
            isinstance(action, Node)
            and action.node_package == 'slam_toolbox'
            and action.node_executable == 'async_slam_toolbox_node'
        )
    ]

    assert len(slam_nodes) == 1
