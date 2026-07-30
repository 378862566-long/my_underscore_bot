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

import pytest
import rclpy
from sensor_msgs.msg import LaserScan


RUN_GAZEBO_TESTS = os.environ.get('RUN_GAZEBO_TESTS') == '1'
GAZEBO_LOG = Path('/tmp/my_underscore_bot_lidar_test.log')


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
    reason='Set RUN_GAZEBO_TESTS=1 to run the Gazebo lidar test.',
)
def test_lidar_publishes_valid_scan(gazebo_simulation):
    """Verify the bridged lidar topic and its scan configuration."""
    rclpy.init()
    node = rclpy.create_node('lidar_test')
    scans = []

    def record_scan(message):
        scans.append(message)

    subscription = node.create_subscription(
        LaserScan,
        '/scan',
        record_scan,
        10,
    )

    try:
        scan_deadline = time.monotonic() + 30.0
        while not scans and time.monotonic() < scan_deadline:
            if gazebo_simulation.poll() is not None:
                pytest.fail(
                    f'Gazebo exited early. See log: {GAZEBO_LOG}'
                )
            rclpy.spin_once(node, timeout_sec=0.1)

        assert scans, f'No /scan message received. See log: {GAZEBO_LOG}'
        scan = scans[-1]
        finite_ranges = [
            range_value
            for range_value in scan.ranges
            if math.isfinite(range_value)
        ]

        assert scan.header.frame_id == 'robot/base_footprint/lidar'
        assert len(scan.ranges) == 360
        assert scan.angle_min == pytest.approx(-math.pi, rel=1e-5)
        assert scan.angle_max == pytest.approx(math.pi, rel=1e-5)
        assert scan.range_min == pytest.approx(0.12, rel=1e-5)
        assert scan.range_max == pytest.approx(8.0)
        assert finite_ranges
        assert all(
            scan.range_min <= range_value <= scan.range_max
            for range_value in finite_ranges
        )
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
