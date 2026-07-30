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
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pytest
import rclpy


RUN_GAZEBO_TESTS = os.environ.get('RUN_GAZEBO_TESTS') == '1'
GAZEBO_LOG = Path('/tmp/my_underscore_bot_motion_test.log')


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
