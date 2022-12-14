import numpy as np
import rclpy
from matplotlib import pyplot as plt
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool


class Lidar_info_node(Node):



    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.distances = []
        self.vAmout = len(self.distances)
        self.confidence = 0.03
        self.create_subscription(Float32MultiArray, '/laser/scanned', self.new_data, 10)
        self.create_subscription(Bool, '/lidar/graph', self.callback, 10)

    def new_data(self, data):
        self.distances = []
        for tRange in data.data:
            self.distances.append(tRange)
        self.vAmout = len(self.distances)

    def callback(self, data):
        self.x = []
        if len(self.distances) == 0:
            pass
        else:
            for i in range(self.vAmout):
                self.x.append(i)
            ausgleich = []
            changedData = self.distances.copy()

            for i in range(self.vAmout):
                ausgleich.append(float(self.distances[i] * 0.8 + self.distances[i-1] * 0.1 + self.distances[(i+1) % self.vAmout] * 0.1))
                changedData[i] -= self.confidence

            for x in self.x:
                if ausgleich[x] < changedData[x]:
                    if changedData[x] != -self.confidence:
                        plt.scatter(x, ausgleich[x], color='red')
                else:
                    if changedData[x] != -self.confidence:
                        plt.scatter(x, ausgleich[x], color='green')
                if changedData[x] != -self.confidence:
                    plt.scatter(x, changedData[x], color='blue')

            plt.show()


def main(args=None):
    rclpy.init(args=args)
    lidar_info_node_instance = Lidar_info_node()
    rclpy.spin(lidar_info_node_instance)
    lidar_info_node_instance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
