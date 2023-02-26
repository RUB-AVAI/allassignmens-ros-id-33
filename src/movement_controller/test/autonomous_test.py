from unittest import TestCase


import numpy as np
import rclpy
from src.movement_controller.movement_controller.autonomous import AutonomousController


class AutonomousTest(TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.autonomous_node = AutonomousController()

    def tearDown(self) -> None:
        self.autonomous_node.destroy_node()

    # =========== UNIT TESTS ===============

    def test_normalize_radian(self):
        self.assertEqual(np.pi, self.autonomous_node.normalize_radians(3*np.pi))
        self.assertEqual(0, self.autonomous_node.normalize_radians(0))