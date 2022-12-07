from unittest import TestCase
import rclpy
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, KeyCode
from src.movement_controller.movement_controller.controller_node import Controller


class ControllerTest(TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.controller = Controller()

    def tearDown(self) -> None:
        self.controller.destroy_node()

    def check_vel_control(self, char):
        pressed = KeyCode.from_char(char)
        self.controller.on_press(pressed)

        if char == 'w':
            self.assertGreater(self.controller.linear_vel, 0.)
        elif char == 's':
            self.assertLess(self.controller.linear_vel, 0.)
        elif char == 'a':
            self.assertGreater(self.controller.angular_vel, 0.)
        elif char == 'd':
            self.assertLess(self.controller.angular_vel, 0.)

        self.controller.on_release(pressed)

        if char in ['w', 's']:
            self.assertEqual(0., self.controller.linear_vel)
        elif char in ['a', 'd']:
            self.assertEqual(0., self.controller.angular_vel)

    # =========== UNIT TESTS ===============

    def test_create_controller(self):
        self.assertEqual(0., self.controller.angular_vel)
        self.assertEqual(0., self.controller.linear_vel)
        self.assertIsInstance(self.controller.pub_speed.msg_type(), Twist)
        self.assertEqual('/cmd_vel', self.controller.pub_speed.topic_name)

    def test_key_commands(self):
        self.check_vel_control('w')
        self.check_vel_control('a')
        self.check_vel_control('s')
        self.check_vel_control('d')


# =========== INTEGRATION TESTS ===============

class ControllerIntegrationTest(TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.controller = Controller()
        self.controller_sub = rclpy.create_node('controller_sub')
        self.received_messages = []
        self.controller_sub.create_subscription(Twist,
                                                '/cmd_vel',
                                                lambda m: self.received_messages.append(m),
                                                10)

    def tearDown(self) -> None:
        self.controller.destroy_node()
        self.controller_sub.destroy_node()

    def check_last_twist(self, angular_expected, linear_expected):
        twist = self.received_messages[-1]
        self.assertIsInstance(twist, Twist)

        self.assertEqual(linear_expected, twist.linear.x)
        self.assertEqual(0., twist.linear.y)
        self.assertEqual(0., twist.linear.z)

        self.assertEqual(0., twist.angular.x)
        self.assertEqual(0., twist.angular.y)
        self.assertEqual(angular_expected, twist.angular.z)

    def test_message_publishing(self):
        self.controller.timer_callback()
        rclpy.spin_once(self.controller_sub)
        self.assertEqual(1, len(self.received_messages))
        self.check_last_twist(self.controller.angular_vel,
                              self.controller.linear_vel)
        self.receives_messages = [0]
