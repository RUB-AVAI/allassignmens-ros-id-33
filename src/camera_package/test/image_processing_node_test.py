from unittest import TestCase

import PIL.Image
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import glob

from src.camera_package.camera_package.image_processing_node import ImageProcessingNode


class ImageProcessingNodeUnitTest(TestCase):

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
class ImageProcessingNodeIntegrationTest(TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.ipNode = ImageProcessingNode()
        self.received_messages_image_sub = []
        self.node_class = Node("test")
        self.cvBridge = CvBridge()
        self.image_publisher_test = self.node_class.create_publisher(Image, "/images/raw", 10)
        self.image_subscriber_test = rclpy.create_node("TestSubscriber")
        self.image_subscriber_test.create_subscription(Image,
                                                       '/images/processed',
                                                       lambda m: self.received_messages_image_sub.append(m),
                                                       10)

    def test_image_publishing_and_resolution(self):
        # GIVEN raw images
        raw_images = [cv2.imread(file) for file in glob.glob("images/raw/*.png")]

        # WHEN node is spinning
        # THEN there have to be 5 images published
        for image in raw_images:
            self.image_publisher_test.publish(self.cvBridge.cv2_to_imgmsg(image))
            rclpy.spin_once(self.ipNode)
            rclpy.spin_once(self.image_subscriber_test)

        rclpy.spin_once(self.ipNode)
        rclpy.spin_once(self.image_subscriber_test)
        self.assertEqual(5, len(self.received_messages_image_sub))

        for image in self.received_messages_image_sub:
            cv2image = self.cvBridge.imgmsg_to_cv2(image)
            self.assertEqual(cv2image.shape, (480, 640, 3))

        self.received_messages_image_sub.clear()

    def tearDown(self) -> None:
        self.ipNode.destroy_node()
        self.image_subscriber_test.destroy_node()
