import random
from unittest import TestCase

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

from src.sensor_fusion_package.fusion_package.cone_localization_node import ConeLocalizationNode


class ConeLocalizationTest(TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.coneNode = ConeLocalizationNode()

    def tearDown(self) -> None:
        self.coneNode.destroy_node()

    def check_euclidiean_coordinates(self, angle, distance):
        rad = angle * (np.pi / 180)
        x = distance * np.cos(-rad)
        y = distance * np.sin(-rad)
        self.assertEqual((x, y), self.coneNode.get_euclidean_coordinates(angle, distance))

    def check_lidar_range(self, cone):
        fov = 62.2
        pictureBeginning = 180 - (fov / 2)
        amount_pixels = 640
        left = pictureBeginning + cone[0] * fov / amount_pixels
        right = pictureBeginning + cone[2] * fov / amount_pixels
        self.assertEqual((left, right), self.coneNode.calculate_lidar_range(cone))


    # ======== UNIT TESTS ============

    def test_topics(self):
        self.assertEqual('/images/labels', self.coneNode.labelsub.topic_name)
        self.assertEqual('/laser/scanned', self.coneNode.lasersub.topic_name)
        self.assertEqual('/lidar/graph', self.coneNode.graphsub.topic_name)

    def test_euclid_coords(self):
        self.check_euclidiean_coordinates(0.55, 30)
        self.check_euclidiean_coordinates(0, 0)
        self.check_euclidiean_coordinates(-1, -1)
        self.check_euclidiean_coordinates(200, 200)

    def test_check_lidar_range(self):
        for i in range(10):
            self.check_lidar_range([random.randint(-200, 200), random.randint(-200, 200), random.randint(-200, 200),
                                    random.randint(-200, 200), 2, 2])
            self.check_lidar_range(
                [random.randrange(0, 1), random.randrange(0, 1), random.randrange(0, 1), random.randrange(0, 1), 0, 0])

   # ========= INTEGRATION TESTS ============

class cone_localization_integration(TestCase):

    @classmethod
    def setUpClass(self) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.localizationNode = ConeLocalizationNode()

        self.test_node = Node("test")
        self.labelPublisherTest = self.test_node.create_publisher(Float32MultiArray, '/images/labels', 10)
        self.lidarPublisherTest = self.test_node.create_publisher(Float32MultiArray, '/laser/scanned', 10)

    def check_lidar_subscription(self, data):
        msg = Float32MultiArray()
        msg.data = data
        self.lidarPublisherTest.publish(msg)
        rclpy.spin_once(self.localizationNode)
        for i in range(360):
            self.assertAlmostEqual(data[i], self.localizationNode.lidar_data[359-i])

    def test_lidar_subscription(self):
        data = []
        for i in range(360):
            data.append(random.random())
        self.check_lidar_subscription(data)

        data = []
        for i in range(360):
            data.append(float(random.randrange(-100, 100)))
        self.check_lidar_subscription(data)


    def tearDown(self) -> None:
        self.localizationNode.destroy_node()
        self.test_node.destroy_node()
