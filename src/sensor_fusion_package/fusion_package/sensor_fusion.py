import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class Fusion_Node(Node):

    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.create_subscription(Float32MultiArray, '/images/labels', self.received_labels, 10)
        self.create_subscription(Float32MultiArray, '/laser/scanned', self.callback, 10)

    def callback(self, data):
        print('here')

    def received_labels(self, data):
        one_d_array = data.data
        ndarray = []
        for entry in one_d_array:
            ndarray.append(entry)

        ndarraytemp = np.asarray(ndarray)
        newArr = ndarraytemp.reshape(int(len(ndarraytemp) / 6), 6)
        print('there')


def main(args=None):
    rclpy.init(args=args)
    fusion_node_instance = Fusion_Node()
    rclpy.spin(fusion_node_instance)
    fusion_node_instance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
