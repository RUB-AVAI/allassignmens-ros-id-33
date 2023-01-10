import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile
import numpy as np


class LidarMockNode(Node):

    def __init__(self):
        super().__init__('lidar_mock_node')
        self.get_logger().info('startup')
        self.lidar_publisher = self.create_publisher(LaserScan, '/scan', qos_profile=qos_profile_sensor_data)
        self.lidar_data = [10 * np.sin(np.pi * (i / 180)) for i in range(360)]
        self.timer = self.create_timer(.1, self.timer_callback)
        self.get_logger().info('startup complete')

    def timer_callback(self):
        self.get_logger().info('hello callback')
        self.lidar_data = self.lidar_data[1:] + self.lidar_data[:1]
        msg = LaserScan()
        msg.ranges = self.lidar_data
        self.lidar_publisher.publish(msg)
        self.get_logger().info('lidar data ->')


def main(args=None):
    rclpy.init(args=args)
    lidar_mock_node = LidarMockNode()
    rclpy.spin(lidar_mock_node)
    lidar_mock_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()