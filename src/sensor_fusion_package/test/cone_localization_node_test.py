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

    def test_clustering(self):
        # Dataset structure is (x, y, _, color) with color as:
        # blue == 0, orange == 1, yellow == 2, else is 3.
        dbscan_data_set = [[x, y, 0, self.color_to_int('blue')]
                           for x in np.linspace(10, 10.2, 25)
                           for y in np.linspace(20, 22.2, 25)]
        dbscan_data_set += [[x, y, 0, self.color_to_int('orange')]
                            for x in np.linspace(30, 30.1, 10)
                            for y in np.linspace(40, 41.2, 10)]
        dbscan_data_set += [[x, y, 0, self.color_to_int('yellow')]
                            for x in np.linspace(100, 100.05, 30)
                            for y in np.linspace(101, 103.10, 30)]
        dbscan_data_set += [[12, 13, 0, self.color_to_int('blue')]]

        x_blue_mean, y_blue_mean = np.mean(dbscan_data_set[0:3][0]), np.mean(dbscan_data_set[0:3][1])
        x_orange_mean, y_orange_mean = np.mean(dbscan_data_set[3:6][0]), np.mean(dbscan_data_set[3:6][1])
        x_yellow_mean, y_yellow_mean = np.mean(dbscan_data_set[6:9][0]), np.mean(dbscan_data_set[6:9][1])

        cones_expected = [[x_blue_mean, y_blue_mean, self.color_to_int('blue')],
                          [x_orange_mean, y_orange_mean, self.color_to_int('orange')],
                          [x_yellow_mean, y_yellow_mean, self.color_to_int('yellow')]]

        new_cone_representation = self.coneNode.use_dbscan(dbscan_data_set, _min_samples=2, _eps=0.1)
        self.assertEqual(3, len(new_cone_representation))

        for i in range(len(new_cone_representation)):
            self.assertAlmostEqual(cones_expected[i][0], new_cone_representation[i][0], places=3)
            self.assertAlmostEqual(cones_expected[i][1], new_cone_representation[i][1], places=3)
            self.assertAlmostEqual(cones_expected[i][2], new_cone_representation[i][3], places=3)

    def color_to_int(self, color: str):
        if color == 'blue':
            return 0
        elif color == 'orange':
            return 1
        elif color == 'yellow':
            return 2
        else:
            return 3


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
            self.assertAlmostEqual(data[i], self.localizationNode.lidar_data[359 - i])

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
