from unittest import TestCase

import PIL.Image
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

from src.camera_package.camera_package.image_processing_node import ImageProcessingNode


class ImageProcessingNodeTest(TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()


    @classmethod
    def tearDown(self) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.ipNode = ImageProcessingNode()

    def tearDown(self) -> None:
        self.ipNode.destroy_node()

    # =========== UNIT TESTS ===============
    def test_topics(self):
        self.assertEqual("/images/processed", self.ipNode.publisher_.topic_name)
        self.assertEqual("/images/raw", self.ipNode.subscription_.topic_name)

    # =========== INTEGRATION TESTS ===============

class camera_node_integration(TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        #rclpy.init()
        pass

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.ipNode = ImageProcessingNode()
        self.received_messages_image_sub = []
        self.node_class = Node("test")
        self.image_publisher_test = self.node_class.create_publisher(Image, "/images/raw", 10)
        self.image_test_sub.create_subscription(Image,
                                            '/images/processed',
                                            lambda m: self.received_messages_image_sub.append(m),
                                            10)

    def tearDown(self) -> None:
        self.ipNode.destroy_node()

