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

import os
import math
from pathlib import Path
import re
import signal
import subprocess
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pytest
import rclpy


RUN_GAZEBO_TESTS = os.environ.get('RUN_GAZEBO_TESTS') == '1'
GAZEBO_LOG = Path('/tmp/my_underscore_bot_motion_test.log')
ROBOT_MODEL_NAME = 'robot'


def quaternion_to_yaw(z_value, w_value):
    """Convert a planar quaternion into a yaw angle."""
    return 2.0 * math.atan2(z_value, w_value)


def normalize_angle(angle):
    """Normalize an angle to the range from -pi to pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def value_from_block(block, field_name, default):
    """Read one numeric field from Gazebo's text-format message."""
    match = re.search(
        rf'^\s*{field_name}:\s*([-+eE0-9.]+)\s*$',
        block,
        re.MULTILINE,
    )
    return float(match.group(1)) if match else default


def get_gazebo_model_pose():
    """Read the robot ground-truth pose from Gazebo Transport."""
    topics_result = subprocess.run(
        ['ign', 'topic', '-l'],
        check=True,
        capture_output=True,
        text=True,
        timeout=5,
    )
    pose_topics = [
        topic
        for topic in topics_result.stdout.splitlines()
        if topic.endswith('/pose/info') and '/dynamic_pose/' not in topic
    ]
    assert pose_topics, 'Gazebo did not publish a world pose topic.'

    pose_result = subprocess.run(
        ['ign', 'topic', '-e', '-n', '1', '-t', pose_topics[0]],
        check=True,
        capture_output=True,
        text=True,
        timeout=5,
    )
    model_marker = f'name: "{ROBOT_MODEL_NAME}"'
    marker_index = pose_result.stdout.find(model_marker)
    assert marker_index >= 0, (
        f'Gazebo pose message did not contain model {ROBOT_MODEL_NAME}.'
    )

    pose_start = pose_result.stdout.rfind('pose {', 0, marker_index)
    pose_end = pose_result.stdout.find('\npose {', marker_index)
    if pose_end < 0:
        pose_end = len(pose_result.stdout)
    model_pose = pose_result.stdout[pose_start:pose_end]

    position_match = re.search(
        r'position \{(.*?)\}',
        model_pose,
        re.DOTALL,
    )
    orientation_match = re.search(
        r'orientation \{(.*?)\}',
        model_pose,
        re.DOTALL,
    )
    assert position_match and orientation_match

    position_block = position_match.group(1)
    orientation_block = orientation_match.group(1)
    x_value = value_from_block(position_block, 'x', 0.0)
    y_value = value_from_block(position_block, 'y', 0.0)
    z_value = value_from_block(orientation_block, 'z', 0.0)
    w_value = value_from_block(orientation_block, 'w', 1.0)

    return x_value, y_value, quaternion_to_yaw(z_value, w_value)


def stop_process_group(process):
    """Stop a launch process and every child process that it created."""
    if process.poll() is not None:
        return

    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
        return
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)

    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


@pytest.fixture
def gazebo_simulation():
    """Start the project simulation and stop it after the test."""
    environment = os.environ.copy()
    if 'DISPLAY' not in environment and Path('/tmp/.X11-unix/X1').exists():
        environment['DISPLAY'] = ':1'

    with GAZEBO_LOG.open('w') as log_file:
        process = subprocess.Popen(
            [
                'ros2',
                'launch',
                'my_underscore_bot',
                'launch_sim.launch.py',
            ],
            env=environment,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        try:
            yield process
        finally:
            stop_process_group(process)


@pytest.mark.skipif(
    not RUN_GAZEBO_TESTS,
    reason='Set RUN_GAZEBO_TESTS=1 to run the Gazebo motion test.',
)
def test_robot_moves_forward(gazebo_simulation):
    """Send a forward command and verify movement through odometry."""
    rclpy.init()
    node = rclpy.create_node('motion_test')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    odometry_positions = []

    def record_odometry(message):
        odometry_positions.append(message.pose.pose.position.x)

    subscription = node.create_subscription(
        Odometry,
        '/diff_cont/odom',
        record_odometry,
        10,
    )

    try:
        odometry_deadline = time.monotonic() + 30.0
        while not odometry_positions and time.monotonic() < odometry_deadline:
            if gazebo_simulation.poll() is not None:
                pytest.fail(
                    f'Gazebo exited early. See log: {GAZEBO_LOG}'
                )
            rclpy.spin_once(node, timeout_sec=0.1)

        assert odometry_positions, (
            f'No odometry received. See log: {GAZEBO_LOG}'
        )
        start_x = odometry_positions[-1]

        forward_command = Twist()
        forward_command.linear.x = 0.2
        motion_deadline = time.monotonic() + 3.0

        while time.monotonic() < motion_deadline:
            publisher.publish(forward_command)
            rclpy.spin_once(node, timeout_sec=0.1)

        stop_command = Twist()
        publisher.publish(stop_command)

        odometry_deadline = time.monotonic() + 2.0
        while time.monotonic() < odometry_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        distance_moved = odometry_positions[-1] - start_x
        assert distance_moved > 0.1, (
            f'Robot moved only {distance_moved:.3f} m; expected over 0.1 m. '
            f'See log: {GAZEBO_LOG}'
        )
    finally:
        publisher.publish(Twist())
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(
    not RUN_GAZEBO_TESTS,
    reason='Set RUN_GAZEBO_TESTS=1 to run the Gazebo motion test.',
)
def test_robot_rotates_in_place(gazebo_simulation):
    """Compare an in-place turn in odometry and Gazebo ground truth."""
    rclpy.init()
    node = rclpy.create_node('rotation_test')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    odometry_poses = []

    def record_odometry(message):
        orientation = message.pose.pose.orientation
        odometry_poses.append((
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            quaternion_to_yaw(orientation.z, orientation.w),
        ))

    subscription = node.create_subscription(
        Odometry,
        '/diff_cont/odom',
        record_odometry,
        10,
    )

    try:
        odometry_deadline = time.monotonic() + 30.0
        while not odometry_poses and time.monotonic() < odometry_deadline:
            if gazebo_simulation.poll() is not None:
                pytest.fail(
                    f'Gazebo exited early. See log: {GAZEBO_LOG}'
                )
            rclpy.spin_once(node, timeout_sec=0.1)

        assert odometry_poses, (
            f'No odometry received. See log: {GAZEBO_LOG}'
        )
        start_yaw = odometry_poses[-1][2]

        turn_command = Twist()
        turn_command.angular.z = 0.5
        motion_deadline = time.monotonic() + 3.0

        while time.monotonic() < motion_deadline:
            publisher.publish(turn_command)
            rclpy.spin_once(node, timeout_sec=0.1)

        publisher.publish(Twist())

        odometry_deadline = time.monotonic() + 1.0
        while time.monotonic() < odometry_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        odometry_yaw = normalize_angle(
            odometry_poses[-1][2] - start_yaw
        )
        gazebo_x, gazebo_y, gazebo_yaw = get_gazebo_model_pose()
        yaw_error = abs(normalize_angle(gazebo_yaw - odometry_yaw))
        position_drift = math.hypot(gazebo_x, gazebo_y)

        assert abs(odometry_yaw) > 0.5, (
            f'Robot rotated only {odometry_yaw:.3f} rad in odometry.'
        )
        assert position_drift < 0.02, (
            f'Robot drifted {position_drift:.3f} m while turning.'
        )
        assert yaw_error < 0.25, (
            f'Gazebo and odometry yaw differ by {yaw_error:.3f} rad.'
        )
    finally:
        publisher.publish(Twist())
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
