import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Pub(Node):

    def __init__(self):
        super().__init__('pub')
        self.ctr = 0
        self.publisher_ = self.create_publisher(String, 'demo', 10)
        self.timer = self.create_timer(0.25, self.callback)

    def callback(self):
        self.ctr = self.ctr + 1
        msg = String()
        msg.data = "%d" % self.ctr
        self.publisher_.publish(msg)
        self.get_logger().info('<- %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    pub = Pub()
    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
