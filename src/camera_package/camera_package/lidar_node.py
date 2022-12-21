import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Bool, String
from rclpy.qos import qos_profile_sensor_data, QoSProfile


class Lidar_node(Node):

    def __init__(self):
        super().__init__('lidarsub')
        self.laserSub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, qos_profile=qos_profile_sensor_data)
        #self.testSub = self.create_subscription(Bool, '/camera/shutter', self.test_callback, 10)
        self.laserProcessed = self.create_publisher(Float32MultiArray, '/laser/scanned', 10)
        self.ranges = [0, 0, 2, 0, 0, 0, 0.4, 0.6, 0.7, 0, 0, 0]
        self.x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
        self.coneAngles = []

    def test_callback(self, msg):
        msg = Float32MultiArray()
        for tRange in self.ranges:
            msg.data.append(tRange)
        self.laserProcessed.publish(msg)

    def lidar_callback(self, msg):
        #self.get_logger().info('-> %s' % msg.ranges)
        message = Float32MultiArray()
        self.ranges = []
        for range in msg.ranges:
            self.ranges.append(range)
            message.data.append(range)
        self.laserProcessed.publish(message)
        self.process_ranges()

    def process_ranges(self):
        pass

        # function here to get cone data


def main(args=None):
    rclpy.init(args=args)
    qos = QoSProfile(depth=10)
    lNode = Lidar_node()
    rclpy.spin(lNode)
    lNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
