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

from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy


RUN_GAZEBO_TESTS = os.environ.get('RUN_GAZEBO_TESTS') == '1'
SLAM_LOG = Path('/tmp/my_underscore_bot_slam_test.log')


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

    # The launch parent can exit before every child in its process group.
    time.sleep(0.2)
    for shutdown_signal in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, shutdown_signal)
        except ProcessLookupError:
            break
        time.sleep(0.5)


@pytest.fixture
def slam_simulation():
    """Start online SLAM and stop every launched process after the test."""
    environment = os.environ.copy()
    if 'DISPLAY' not in environment and Path('/tmp/.X11-unix/X1').exists():
        environment['DISPLAY'] = ':1'

    with SLAM_LOG.open('w') as log_file:
        process = subprocess.Popen(
            [
                'ros2',
                'launch',
                'my_underscore_bot',
                'slam.launch.py',
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
    reason='Set RUN_GAZEBO_TESTS=1 to run the Gazebo SLAM test.',
)
def test_slam_publishes_occupancy_grid(slam_simulation):
    """Verify that SLAM consumes scans and publishes a valid map."""
    rclpy.init()
    node = rclpy.create_node('slam_test')
    maps = []

    def record_map(message):
        maps.append(message)

    subscription = node.create_subscription(
        OccupancyGrid,
        '/map',
        record_map,
        10,
    )

    try:
        map_deadline = time.monotonic() + 30.0
        while not maps and time.monotonic() < map_deadline:
            if slam_simulation.poll() is not None:
                pytest.fail(
                    f'SLAM exited early. See log: {SLAM_LOG}'
                )
            rclpy.spin_once(node, timeout_sec=0.1)

        assert maps, f'No /map message received. See log: {SLAM_LOG}'
        occupancy_grid = maps[-1]
        known_cells = [
            cell
            for cell in occupancy_grid.data
            if cell >= 0
        ]

        assert occupancy_grid.header.frame_id == 'map'
        assert occupancy_grid.info.resolution == pytest.approx(0.05)
        assert occupancy_grid.info.width > 0
        assert occupancy_grid.info.height > 0
        assert len(occupancy_grid.data) == (
            occupancy_grid.info.width * occupancy_grid.info.height
        )
        assert known_cells
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
