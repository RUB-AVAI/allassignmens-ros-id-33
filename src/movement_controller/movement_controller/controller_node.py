import rclpy
from rclpy.node import Node
from pynput import keyboard
from geometry_msgs.msg import Twist

PUBLISHER_PERIOD = .5

class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.angular_vel = 0.
        self.linear_vel = 0.
        self.pub_speed = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(PUBLISHER_PERIOD, self.timer_callback)
        self.keyboard_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.keyboard_listener.start()
        self.get_logger().info("startup complete")

    def on_press(self, key):
        try:
            char = getattr(key, 'char', None)
            if char in ['w', 'a', 's', 'd']:
                if char == 'w':
                    self.linear_vel = .2
                elif char == 's':
                    self.linear_vel = -.2
                elif char == 'a':
                    self.angular_vel = 1.
                elif char == 'd':
                    self.angular_vel = -1.

        except Exception as e:
            self.get_logger().error(str(e))

    def on_release(self, key):
        try:
            char = getattr(key, 'char', None)
            if char in ['w', 's']:
                self.linear_vel = 0.
            elif char in ['a', 'd']:
                self.angular_vel = 0.

        except Exception as e:
            self.get_logger().error(str(e))

    def timer_callback(self):
        # if some speed changed, publish new
        self.get_logger().info(f"lin_vel={self.linear_vel}, ang_vel={self.angular_vel}")
        # calculate message and publish
        move_cmd = Twist()
        move_cmd.linear.x = float(self.linear_vel)
        move_cmd.angular.z = float(self.angular_vel)
        self.pub_speed.publish(move_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()