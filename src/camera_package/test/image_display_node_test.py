import sys
from unittest import TestCase

import rclpy
from PyQt5 import QtWidgets
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool

from src.camera_package.camera_package.hmi import Ui_MainWindow
from src.camera_package.camera_package.image_display_node import ImageDisplayNode


class ImageDisplayNodeTest(TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDown(self) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        app = QtWidgets.QApplication(sys.argv)
        hmi = QtWidgets.QMainWindow()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(hmi)
        self.idnode = ImageDisplayNode(self.ui)

    def tearDown(self) -> None:
        self.idnode.destroy_node()

    # =========== UNIT TESTS ===============
    def test_topics(self):
        self.assertEqual("/images/processed", self.idnode.processed_images_subscription_.topic_name)
        self.assertEqual("/camera/freq", self.idnode.camera_ctrl_publisher_.topic_name)
        self.assertEqual("/camera/shutter", self.idnode.shutter_publisher_.topic_name)


# =========== INTEGRATION TESTS ===============

class camera_node_integration(TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        # rclpy.init()
        pass

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        app = QtWidgets.QApplication(sys.argv)
        hmi = QtWidgets.QMainWindow()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(hmi)
        self.idnode = ImageDisplayNode(self.ui)

        self.node_class = Node("test")
        self.image_publisher_test = self.node_class.create_publisher(Image, "/images/raw", 10)

        #frequency subscriber
        self.frequency_sub = rclpy.create_node('freq_sub')
        self.received_messages_freq = []
        self.frequency_sub.create_subscription(Float64,
                                                '/camera/freq',
                                               lambda m: self.received_messages_freq.append(m),
                                               10)

        #shutter sub
        self.shutter_sub = rclpy.create_node('shutter_sub')
        self.received_messages_shutter = []
        self.shutter_sub.create_subscription(Bool,
                                             '/camera/shutter',
                                             lambda m: self.received_messages_shutter.append(m),
                                             10)

    def tearDown(self) -> None:
        self.idnode.destroy_node()

    def check_new_freq(self, freq):
        self.idnode.current_freq = freq
        self.idnode.camera_freq_publisher_callback_()
        rclpy.spin_once(self.frequency_sub)
        if freq == 0:
            checkValue = 0
        else:
            checkValue = 1 / freq

        self.assertEqual(checkValue, self.received_messages_freq[-1].data)

    def test_new_freq(self):

        self.check_new_freq(5)
        self.check_new_freq(10)
        self.check_new_freq(5000)
        self.check_new_freq(0)
        self.check_new_freq(-10)

    def test_shutter(self):
        self.idnode.camera_shutter_callback_()
        rclpy.spin_once(self.shutter_sub)
        self.assertEqual(True, self.received_messages_shutter[-1].data)
