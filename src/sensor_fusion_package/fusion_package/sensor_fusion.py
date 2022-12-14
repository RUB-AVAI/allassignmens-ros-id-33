import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan


class Fusion_Node(Node):

    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.create_subscription(Float32MultiArray, '/images/labels', self.received_labels, 10)
        self.create_subscription(LaserScan, '/laser/scanned', self.message, 10)


    def message(self, data):
        print(data.data)


    def received_labels(self, data):
        one_d_array = data.data
        ndarray = []
        print(one_d_array)
        for entry in one_d_array:
            ndarray.append(entry)

        ndarraytemp = np.asarray(ndarray)
        print(ndarraytemp)
        newArr = ndarraytemp.reshape(int(len(ndarraytemp) / 6), 6)


def main(args=None):
    rclpy.init(args=args)
    fusion_node_instance = Fusion_Node()
    rclpy.spin(fusion_node_instance)
    fusion_node_instance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
