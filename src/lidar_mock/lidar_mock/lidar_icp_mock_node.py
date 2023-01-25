import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile
import numpy as np


class LidarICPMockNode(Node):

    def __init__(self):
        super().__init__('lidar_icp_mock_node')
        self.get_logger().info('startup of icp publishing')
        self.lidar_publisher = self.create_publisher(LaserScan, '/scan', qos_profile=qos_profile_sensor_data)
        self.lidar_data = np.random.randint(3.5, size=360)
        self.get_logger().info('startup complete')

    def timer_callback(self):
        self.get_logger().info('hello callback')
        msg = LaserScan()
        msg.ranges = self.lidar_data
        self.lidar_publisher.publish(msg)
        self.get_logger().info('lidar data ->')

    def publish_lidar_data(self):
        self.get_logger().info('publishing mock lidar')
        msg = LaserScan()
        msg.ranges = self.lidar_data
        self.lidar_publisher.publish(msg)
        self.get_logger().info('lidar data ->')
        self.get_logger().info(self.lidar_data[:5])

    def process_rotation(self, angle):
        self.lidar_data = self.lidar_data[angle:] + self.lidar_data[:angle]

    def process_movement(self, movement):
        new_lidar_data = [0] * 360
        for angle in range(len(self.lidar_data)):
            a = self.lidar_data[angle] * math.sin(math.pi * (angle / 180))
            b = self.lidar_data[angle] * math.cos(math.pi * (angle / 180))
            angle_new_tan = (a - movement) / b
            angle_new = math.atan(angle_new_tan)
            new_range = b / math.cos(angle_new)
            angle_new = round((angle_new / math.pi) * 180)
            new_lidar_data[angle_new] = new_range
        self.lidar_data = new_lidar_data
