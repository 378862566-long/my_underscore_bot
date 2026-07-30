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

from pathlib import Path
import xml.etree.ElementTree as ElementTree

import xacro


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROBOT_XACRO = PACKAGE_ROOT / 'description' / 'robot.urdf.xacro'


def expand_robot_xacro():
    """Expand the main Xacro file and return its XML root element."""
    robot_xml = xacro.process_file(str(ROBOT_XACRO)).toxml()
    return ElementTree.fromstring(robot_xml)


def test_xacro_expands_to_robot():
    robot = expand_robot_xacro()

    assert robot.tag == 'robot'
    assert robot.attrib['name'] == 'robot'


def test_required_links_exist():
    robot = expand_robot_xacro()
    link_names = {link.attrib['name'] for link in robot.findall('link')}

    expected_links = {
        'base_footprint',
        'base_link',
        'chassis',
        'left_wheel',
        'right_wheel',
        'front_caster',
    }

    assert expected_links.issubset(link_names)


def test_required_joints_exist():
    robot = expand_robot_xacro()
    joint_names = {joint.attrib['name'] for joint in robot.findall('joint')}

    expected_joints = {
        'base_footprint_joint',
        'chassis_joint',
        'left_wheel_joint',
        'right_wheel_joint',
        'front_caster_joint',
    }

    assert expected_joints.issubset(joint_names)


def test_ros2_control_is_present():
    robot = expand_robot_xacro()

    assert robot.find('ros2_control') is not None
