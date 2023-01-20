import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile


class Fusion_Node(Node):

    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.create_subscription(Float32MultiArray, '/images/labels', self.received_labels, 10)
        self.create_subscription(LaserScan, 'scan', self.lidar_callback, qos_profile=qos_profile_sensor_data)

    def lidar_callback(self, msg):
        ranges = msg.ranges
        print(ranges)

    def received_labels(self, data):
        one_d_array = data.data
        ndarray = []
        for entry in one_d_array:
            ndarray.append(entry)

        ndarraytemp = np.asarray(ndarray)
        newArr = ndarraytemp.reshape(int(len(ndarraytemp) / 6), 6)


def main(args=None):
    rclpy.init(args=args)
    fusion_node_instance = Fusion_Node()
    rclpy.spin(fusion_node_instance)
    fusion_node_instance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
