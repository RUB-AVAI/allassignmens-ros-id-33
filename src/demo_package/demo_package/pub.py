# Copyright 2015 Open Source Robotics Foundation, Inc.
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
from rclpy.node import Node
from std_msgs.msg import String


class Pub(Node):

    def __init__(self):
        super().__init__('pub')
        self.ctr = 0
        self.publisher_ = self.create_publisher(String, 'demo', 10)
        self.timer = self.create_timer(0.25, self.callback)

    def callback(self):
        self.ctr = self.ctr + 1
        msg = String()
        msg.data = '%d' % self.ctr
        self.publisher_.publish(msg)
        self.get_logger().info('<- %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    pub = Pub()
    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
