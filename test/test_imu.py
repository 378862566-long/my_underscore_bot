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

import math
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import Twist
import pytest
import rclpy
from sensor_msgs.msg import Imu


RUN_GAZEBO_TESTS = os.environ.get('RUN_GAZEBO_TESTS') == '1'
GAZEBO_LOG = Path('/tmp/my_underscore_bot_imu_test.log')


def stop_process_group(process):
    """Stop a launch process and every child process that it created."""
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

    time.sleep(0.2)
    for shutdown_signal in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, shutdown_signal)
        except ProcessLookupError:
            break
        time.sleep(0.5)


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
    reason='Set RUN_GAZEBO_TESTS=1 to run the Gazebo IMU test.',
)
def test_imu_reports_gravity_and_rotation(gazebo_simulation):
    """Verify stationary acceleration and yaw rate during rotation."""
    rclpy.init()
    node = rclpy.create_node('imu_test')
    messages = []

    subscription = node.create_subscription(
        Imu,
        '/imu',
        messages.append,
        20,
    )
    command_publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    try:
        message_deadline = time.monotonic() + 30.0
        while len(messages) < 10 and time.monotonic() < message_deadline:
            if gazebo_simulation.poll() is not None:
                pytest.fail(
                    f'Gazebo exited early. See log: {GAZEBO_LOG}'
                )
            rclpy.spin_once(node, timeout_sec=0.1)

        assert messages, f'No /imu message received. See log: {GAZEBO_LOG}'
        stationary = messages[-1]
        orientation_norm = math.sqrt(
            stationary.orientation.x ** 2
            + stationary.orientation.y ** 2
            + stationary.orientation.z ** 2
            + stationary.orientation.w ** 2
        )

        assert stationary.header.frame_id == (
            'robot/base_footprint/imu_sensor'
        )
        assert orientation_norm == pytest.approx(1.0, abs=0.02)
        assert abs(stationary.angular_velocity.z) < 0.05
        assert stationary.linear_acceleration.z == pytest.approx(
            9.8,
            abs=0.3,
        )

        messages.clear()
        rotation_command = Twist()
        rotation_command.angular.z = 0.5
        rotation_deadline = time.monotonic() + 4.0

        while time.monotonic() < rotation_deadline:
            command_publisher.publish(rotation_command)
            rclpy.spin_once(node, timeout_sec=0.05)

        stop_command = Twist()
        for _ in range(5):
            command_publisher.publish(stop_command)
            rclpy.spin_once(node, timeout_sec=0.05)

        measured_yaw_rates = [
            message.angular_velocity.z
            for message in messages
        ]

        assert measured_yaw_rates
        assert max(measured_yaw_rates) > 0.2
    finally:
        node.destroy_subscription(subscription)
        node.destroy_publisher(command_publisher)
        node.destroy_node()
        rclpy.shutdown()
