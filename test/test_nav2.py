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
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NAVIGATION_LAUNCH_FILE = (
    PACKAGE_ROOT / 'launch' / 'navigation.launch.py'
)
NAV2_PARAMS_FILE = PACKAGE_ROOT / 'config' / 'nav2_params.yaml'
MAP_FILE = PACKAGE_ROOT / 'maps' / 'my_map.yaml'


def load_navigation_launch_description():
    """Load and generate the Nav2 launch description."""
    module_spec = importlib.util.spec_from_file_location(
        'navigation_launch',
        NAVIGATION_LAUNCH_FILE,
    )
    launch_module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(launch_module)
    return launch_module.generate_launch_description()


def load_yaml(path):
    """Load a YAML file used by the navigation stack."""
    with path.open(encoding='utf-8') as yaml_file:
        return yaml.safe_load(yaml_file)


def test_navigation_launch_file_generates_description():
    """Verify that the navigation launch file can be evaluated."""
    launch_description = load_navigation_launch_description()

    assert isinstance(launch_description, LaunchDescription)


def test_navigation_launch_arguments_are_declared():
    """Verify that map, Nav2, and simulation settings are configurable."""
    launch_description = load_navigation_launch_description()
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
        'map',
        'params_file',
        'autostart',
        'rviz',
        'rviz_config',
    }

    assert expected_arguments.issubset(argument_names)


def test_saved_map_configuration_is_complete():
    """Verify that the saved map YAML points to an existing image."""
    map_parameters = load_yaml(MAP_FILE)
    image_file = MAP_FILE.parent / map_parameters['image']

    assert image_file.is_file()
    assert map_parameters['resolution'] == 0.05
    assert len(map_parameters['origin']) == 3


def test_nav2_uses_robot_frames_and_odometry():
    """Verify that Nav2 matches the robot TF and odometry interfaces."""
    parameters = load_yaml(NAV2_PARAMS_FILE)
    navigator = parameters['bt_navigator']['ros__parameters']
    local_costmap = parameters[
        'local_costmap'
    ]['local_costmap']['ros__parameters']
    global_costmap = parameters[
        'global_costmap'
    ]['global_costmap']['ros__parameters']

    assert navigator['global_frame'] == 'map'
    assert navigator['robot_base_frame'] == 'base_footprint'
    assert navigator['odom_topic'] == '/diff_cont/odom'
    assert local_costmap['global_frame'] == 'odom'
    assert local_costmap['robot_base_frame'] == 'base_footprint'
    assert global_costmap['global_frame'] == 'map'
    assert global_costmap['robot_base_frame'] == 'base_footprint'


def test_amcl_matches_lidar_configuration():
    """Verify that AMCL consumes the simulated lidar with matching limits."""
    parameters = load_yaml(NAV2_PARAMS_FILE)
    amcl = parameters['amcl']['ros__parameters']

    assert amcl['scan_topic'] == 'scan'
    assert amcl['base_frame_id'] == 'base_footprint'
    assert amcl['odom_frame_id'] == 'odom'
    assert amcl['global_frame_id'] == 'map'
    assert amcl['laser_min_range'] == 0.12
    assert amcl['laser_max_range'] == 8.0


def test_costmaps_use_polygon_footprint():
    """Verify that both costmaps use the calibrated rectangular footprint."""
    parameters = load_yaml(NAV2_PARAMS_FILE)
    expected_footprint = (
        '[[-0.12, -0.20], [-0.12, 0.20], '
        '[0.22, 0.20], [0.22, -0.20]]'
    )

    for costmap_name in ('local_costmap', 'global_costmap'):
        costmap = parameters[
            costmap_name
        ][costmap_name]['ros__parameters']

        assert costmap['footprint'] == expected_footprint
        assert 'robot_radius' not in costmap
