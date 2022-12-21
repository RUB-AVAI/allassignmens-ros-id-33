from unittest import TestCase


import rclpy
import sensor_msgs.msg
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool
from cv_bridge import CvBridge

from src.camera_package.camera_package.camera_node import CameraNode


class CameraNodeTest(TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDown(self) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.cameraNode = CameraNode()

    def tearDown(self) -> None:
        self.cameraNode.destroy_node()

    def check_timer(self, ms):
        msg = Float64()
        msg.data = float(ms)
        self.cameraNode.camera_ctrl_callback(msg)
        nsTime = ms * 1000000000
        self.assertEqual(nsTime, self.cameraNode.timer.timer_period_ns)

    # =========== UNIT TESTS ===============

    def test_video_capture(self):
        self.assertEqual(True, self.cameraNode.cap.isOpened())

    def test_topics(self):
        self.assertEqual("/images/raw", self.cameraNode.publisher_.topic_name)
        self.assertEqual("/camera/freq", self.cameraNode.subscription_.topic_name)
        self.assertEqual("/camera/shutter", self.cameraNode.shutter_subscription.topic_name)

    def test_timer(self):
        self.check_timer(10)
        self.check_timer(10)
        self.check_timer(1000)
        #self.check_timer(-1)

    # =========== INTEGRATION TESTS ===============


class camera_node_integration(TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        pass

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.camera_node = CameraNode()
        self.image_test_sub = rclpy.create_node('image_test_sub')
        self.bridge = CvBridge()
        self.received_messages_image_sub = []
        self.image_test_sub.create_subscription(Image,
                                            '/images/raw',
                                            lambda m: self.received_messages_image_sub.append(m),
                                            10)
        self.freq_pub_node = rclpy.node.Node("freqNode")
        self.freq_publisher = self.freq_pub_node.create_publisher(Float64, "/camera/freq", 10)

    def tearDown(self) -> None:
        self.image_test_sub.destroy_node()
        self.camera_node.destroy_node()

    def test_image_publishing(self):
        msg = Bool
        msg.data = True
        self.camera_node.camera_shutter_callback(msg)
        rclpy.spin_once(self.image_test_sub)
        opencvImage = self.bridge.imgmsg_to_cv2(self.received_messages_image_sub[-1])

        self.assertEqual(1, len(self.received_messages_image_sub))
        self.assertEqual(sensor_msgs.msg._image.Image, type(self.received_messages_image_sub[-1]))
        self.assertEqual((1080, 1920, 3), opencvImage.shape)

    def test_freq_change(self):
        msg = Float64()
        msg.data = 10.
        self.freq_publisher.publish(msg)
        print(self.camera_node.timer_period)
        # self.assertEqual(frequency, self.camera_node.timer_period)

