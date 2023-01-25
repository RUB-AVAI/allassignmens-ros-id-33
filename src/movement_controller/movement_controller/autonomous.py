import rclpy
from rclpy.node import Node
import numpy as np

from avai_messages.msg import Track
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class AutonomousController(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.track_subscriber = self.create_subscription(Track, '/track', self.received_track_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.received_odom_callback, 10)

        self.start_position = None

        # TODO: get next target point and keep track of robots orientation to it
        # TODO: while not having reached destination, add constant linar velocity and drive forward
        # TODO: publish in received odom_callback

    def received_track_callback(self, track):
        self.get_logger().info("<- new track")

    def received_odom_callback(self, odom):
        self.get_logger().info("<- new odom data")

    @staticmethod
    def target_dynamic(robot_orientation, target_orientation, turning_speed):
        # to avoid being stuck at 180deg relative to the target. Because rate of change directly on a repeller is 0.
        if abs(AutonomousController.normalize_radians(robot_orientation - target_orientation) - np.pi) < 0.01:
            return np.pi * .25
        return -turning_speed * np.sin(robot_orientation - target_orientation)

    @staticmethod
    def normalize_radians(radians):
        rad = radians
        if rad > np.pi:
            rad -= 2. * np.pi
        elif rad < -np.pi:
            rad += 2. * np.pi
        return rad


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
