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

import pytest
import xacro


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROBOT_XACRO = PACKAGE_ROOT / 'description' / 'robot.urdf.xacro'


def expand_robot_xacro():
    """Expand the main Xacro file and return its XML root element."""
    robot_xml = xacro.process_file(str(ROBOT_XACRO)).toxml()
    return ElementTree.fromstring(robot_xml)


def inertial_values(robot, link_name):
    """Return mass and diagonal inertia values for one link."""
    inertial = robot.find(f"./link[@name='{link_name}']/inertial")
    assert inertial is not None

    mass = float(inertial.find('mass').attrib['value'])
    inertia = inertial.find('inertia')
    diagonal = tuple(
        float(inertia.attrib[axis])
        for axis in ('ixx', 'iyy', 'izz')
    )
    return mass, diagonal


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
        'laser_frame',
        'imu_link',
        'camera_link',
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
        'laser_joint',
        'imu_joint',
        'camera_joint',
    }

    assert expected_joints.issubset(joint_names)


def test_ros2_control_is_present():
    robot = expand_robot_xacro()

    assert robot.find('ros2_control') is not None


def test_teaching_model_mass_distribution():
    """Verify the agreed mass of every physical robot link."""
    robot = expand_robot_xacro()
    expected_masses = {
        'chassis': 0.5,
        'left_wheel': 0.1,
        'right_wheel': 0.1,
        'front_caster': 0.1,
        'laser_frame': 0.05,
        'imu_link': 0.02,
        'camera_link': 0.03,
    }

    actual_masses = {
        link_name: inertial_values(robot, link_name)[0]
        for link_name in expected_masses
    }

    assert actual_masses == pytest.approx(expected_masses)
    base_mass = sum(
        actual_masses[link_name]
        for link_name in (
            'chassis',
            'left_wheel',
            'right_wheel',
            'front_caster',
        )
    )

    assert base_mass == pytest.approx(0.8)
    assert sum(actual_masses.values()) == pytest.approx(0.90)


def test_teaching_model_inertia_values():
    """Verify inertias calculated from the teaching model geometry."""
    robot = expand_robot_xacro()
    expected_inertias = {
        'chassis': (0.0046875, 0.0046875, 0.0075),
        'left_wheel': (
            0.00007583333333333334,
            0.00007583333333333334,
            0.000125,
        ),
        'right_wheel': (
            0.00007583333333333334,
            0.00007583333333333334,
            0.000125,
        ),
        'front_caster': (0.0001, 0.0001, 0.0001),
        'laser_frame': (0.00002375, 0.00002375, 0.00004),
        'imu_link': (
            0.0000016666666666666667,
            0.0000028333333333333335,
            0.000004166666666666667,
        ),
        'camera_link': (0.00000625, 0.0000085, 0.00001025),
    }

    for link_name, expected_diagonal in expected_inertias.items():
        actual_diagonal = inertial_values(robot, link_name)[1]
        assert actual_diagonal == pytest.approx(expected_diagonal)


def test_inertia_values_are_physically_valid():
    """Verify positive moments and the inertia triangle inequalities."""
    robot = expand_robot_xacro()
    physical_links = (
        'chassis',
        'left_wheel',
        'right_wheel',
        'front_caster',
        'laser_frame',
        'imu_link',
        'camera_link',
    )

    for link_name in physical_links:
        _, (ixx, iyy, izz) = inertial_values(robot, link_name)

        assert ixx > 0.0
        assert iyy > 0.0
        assert izz > 0.0
        assert ixx + iyy >= izz
        assert ixx + izz >= iyy
        assert iyy + izz >= ixx


def test_lidar_sensor_configuration():
    """Verify the simulated lidar scan settings."""
    robot = expand_robot_xacro()
    lidar_sensor = robot.find(
        "./gazebo[@reference='laser_frame']/sensor[@name='lidar']"
    )

    assert lidar_sensor is not None
    assert lidar_sensor.attrib['type'] == 'gpu_lidar'
    assert lidar_sensor.findtext('topic') == 'scan'
    assert float(lidar_sensor.findtext('update_rate')) == pytest.approx(10.0)

    lidar = lidar_sensor.find('lidar')
    horizontal_scan = lidar.find('scan/horizontal')
    scan_range = lidar.find('range')

    assert int(horizontal_scan.findtext('samples')) == 360
    assert float(horizontal_scan.findtext('min_angle')) == pytest.approx(
        -3.141592653589793
    )
    assert float(horizontal_scan.findtext('max_angle')) == pytest.approx(
        3.141592653589793
    )
    assert float(scan_range.findtext('min')) == pytest.approx(0.12)
    assert float(scan_range.findtext('max')) == pytest.approx(8.0)


def test_imu_sensor_configuration():
    """Verify the simulated IMU topic and update rate."""
    robot = expand_robot_xacro()
    imu_sensor = robot.find(
        "./gazebo[@reference='imu_link']/sensor[@name='imu_sensor']"
    )

    assert imu_sensor is not None
    assert imu_sensor.attrib['type'] == 'imu'
    assert imu_sensor.findtext('topic') == 'imu'
    assert float(imu_sensor.findtext('update_rate')) == pytest.approx(50.0)


def test_camera_sensor_configuration():
    """Verify the simulated RGB camera settings."""
    robot = expand_robot_xacro()
    camera_sensor = robot.find(
        "./gazebo[@reference='camera_link']"
        "/sensor[@name='camera_sensor']"
    )

    assert camera_sensor is not None
    assert camera_sensor.attrib['type'] == 'camera'
    assert camera_sensor.findtext('topic') == 'camera/image_raw'
    assert float(camera_sensor.findtext('update_rate')) == pytest.approx(15.0)

    camera = camera_sensor.find('camera')
    assert int(camera.findtext('image/width')) == 320
    assert int(camera.findtext('image/height')) == 240
    assert camera.findtext('image/format') == 'R8G8B8'
    assert float(camera.findtext('clip/near')) == pytest.approx(0.1)
    assert float(camera.findtext('clip/far')) == pytest.approx(20.0)
