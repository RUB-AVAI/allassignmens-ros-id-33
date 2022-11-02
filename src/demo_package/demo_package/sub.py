import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Sub(Node):

    def __init__(self):
        super().__init__('sub')
        self.create_subscription(String, 'demo', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info('-> %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    sub = Sub()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
