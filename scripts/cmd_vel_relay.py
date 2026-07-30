#!/usr/bin/env python3

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

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class CmdVelRelay(Node):
    """Relay standard keyboard velocity commands to the diff controller."""

    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.publisher = self.create_publisher(
            TwistStamped, '/diff_cont/cmd_vel', 10)
        self.simulation_time = None
        self.odometry_subscription = self.create_subscription(
            Odometry, '/diff_cont/odom', self.update_simulation_time, 10)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.relay_command, 10)
        self.get_logger().info(
            'Relaying /cmd_vel to stamped /diff_cont/cmd_vel')

    def update_simulation_time(self, message):
        self.simulation_time = message.header.stamp

    def relay_command(self, message):
        if self.simulation_time is None:
            self.get_logger().warning(
                'Waiting for controller odometry time before relaying command',
                throttle_duration_sec=2.0)
            return
        stamped_message = TwistStamped()
        stamped_message.header.stamp = self.simulation_time
        stamped_message.twist = message
        self.publisher.publish(stamped_message)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
