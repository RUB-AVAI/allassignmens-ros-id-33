import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Bool
import matplotlib.pyplot as plt
import numpy as np


class Lidar_node(Node):

    def __init__(self):
        super().__init__('sub')
        self.laserSub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.testSub = self.create_subscription(Bool, '/camera/shutter', self.test_callback, 10)
        self.laserProcessed = self.create_publisher(Float32MultiArray, '/laser/scanned', 10)
        self.ranges = [0, 0, 0, 0, 0, 0, 0.4, 0.6, 0.7, 0, 0, 0]
        self.x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
        self.coneAngles = []


    def test_callback(self, msg):

        self.process_ranges()

    def callback(self, msg):
        self.get_logger().info('-> %s' % msg.data)
        message = Float32MultiArray()
        for range in msg.ranges:
            self.ranges.append(range)
        self.laserProcessed.publish(message)

    def process_ranges(self):
        print("test")
        for x in self.x:
            plt.scatter(x, self.ranges[x])
        plt.show()
        # function here to get cone data


def main(args=None):
    rclpy.init(args=args)

    lNode = Lidar_node()
    rclpy.spin(lNode)

    lNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
