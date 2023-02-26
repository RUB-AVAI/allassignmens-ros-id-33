from unittest import TestCase

import numpy as np
import rclpy
import tf_transformations
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from rclpy.node import Node


from src.movement_controller.movement_controller.path_integration import PathIntegration

class path_integration_test(TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDown(self) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.path_integration_node = PathIntegration()

    def tearDown(self) -> None:
        self.path_integration_node.destroy_node()

        # =========== UNIT TESTS ===============

    def check_offset(self, x_offset, y_offset):
        previous_x = self.path_integration_node.odom.pose.pose.position.x
        previous_y = self.path_integration_node.odom.pose.pose.position.y
        offset_data = Float64MultiArray
        offset_data.data = [x_offset, y_offset]
        self.path_integration_node.calibration_callback(offset_data)

        self.assertEqual(previous_x - x_offset, self.path_integration_node.odom.pose.pose.position.x)
        self.assertEqual(previous_y - y_offset, self.path_integration_node.odom.pose.pose.position.y)

    def check_velocity_callback(self, lin, rot):
        data = Twist()
        data.linear.x = float(lin)
        data.angular.z = float(rot)
        self.path_integration_node.vel_callback(data)
        self.assertEqual(lin, self.path_integration_node.vel.linear.x)
        self.assertEqual(rot, self.path_integration_node.vel.angular.z)

    def check_timer_calculation(self, lin, rot, amout_of_callbacks):
        data = Twist()
        data.linear.x = float(lin)
        data.angular.z = float(rot)
        self.path_integration_node.vel = data
        delta_lin = lin*self.path_integration_node.delta_t
        delta_rot = rot*self.path_integration_node.delta_t
        for i in range(amout_of_callbacks):
            self.path_integration_node.timer_callback()
        self.assertAlmostEqual(delta_lin*amout_of_callbacks, self.path_integration_node.odom.pose.pose.position.x, delta=0.05)
        _, _, yaw = tf_transformations.euler_from_quaternion([self.path_integration_node.odom.pose.pose.orientation.x, self.path_integration_node.odom.pose.pose.orientation.y, self.path_integration_node.odom.pose.pose.orientation.z, self.path_integration_node.odom.pose.pose.orientation.w])
        self.assertAlmostEqual(np.sin(delta_rot*amout_of_callbacks + np.pi), np.sin(yaw), delta=0.05)

    def test_topics(self):
        self.assertEqual("/cmd_vel", self.path_integration_node.vel_subscriber.topic_name)
        self.assertEqual("/codom", self.path_integration_node.position_publisher.topic_name)
        self.assertEqual('/position_calibration', self.path_integration_node.calibration_subscriber.topic_name)

    def test_calibration_offset(self):
        self.check_offset(5, 5)
        self.check_offset(0, 4987)
        self.check_offset(-13085345, 0)
        self.check_offset(0, 0)
        self.check_offset(0.5937, 0.98)

    def test_velocity_callback(self):
        self.check_velocity_callback(0, 0)
        self.check_velocity_callback(20, 0)
        self.check_velocity_callback(0, 20)
        self.check_velocity_callback(-12039, -98321)

    def test_timer_calculation(self):
        self.check_timer_calculation(0, 0.2, 1)
        self.check_timer_calculation(0, 0.2, 3)
        self.check_timer_calculation(1, 0, 1)
        self.check_timer_calculation(1, 0, 3)