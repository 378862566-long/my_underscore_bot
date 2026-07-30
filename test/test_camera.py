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

import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


RUN_GAZEBO_TESTS = os.environ.get('RUN_GAZEBO_TESTS') == '1'
GAZEBO_LOG = Path('/tmp/my_underscore_bot_camera_test.log')


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
    reason='Set RUN_GAZEBO_TESTS=1 to run the Gazebo camera test.',
)
def test_camera_publishes_valid_rgb_image(gazebo_simulation):
    """Verify the bridged camera image dimensions and pixel data."""
    rclpy.init()
    node = rclpy.create_node('camera_test')
    images = []

    subscription = node.create_subscription(
        Image,
        '/camera/image_raw',
        images.append,
        qos_profile_sensor_data,
    )

    try:
        image_deadline = time.monotonic() + 30.0
        while not images and time.monotonic() < image_deadline:
            if gazebo_simulation.poll() is not None:
                pytest.fail(
                    f'Gazebo exited early. See log: {GAZEBO_LOG}'
                )
            rclpy.spin_once(node, timeout_sec=0.1)

        assert images, f'No camera image received. See log: {GAZEBO_LOG}'
        image = images[-1]

        assert image.header.frame_id == (
            'robot/base_footprint/camera_sensor'
        )
        assert image.width == 320
        assert image.height == 240
        assert image.encoding == 'rgb8'
        assert image.step == 320 * 3
        assert len(image.data) == image.step * image.height
        assert any(pixel_value > 0 for pixel_value in image.data)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
