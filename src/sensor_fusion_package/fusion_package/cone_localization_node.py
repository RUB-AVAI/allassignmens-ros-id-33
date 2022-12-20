import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
from matplotlib import pyplot as plt

class ConeLocalizationNode(Node):

    def __init__(self):
        super().__init__('cone_localization_node')
        self.lidar_data = np.asarray([0 for _ in range(360)])
        self.cones = []
        self.create_subscription(Float32MultiArray, '/images/labels', self.received_labels, 10)
        self.create_subscription(Float32MultiArray, '/laser/scanned', self.received_lidar_data, 10)
        self.create_subscription(Bool, '/lidar/graph', self.draw_callback, 10)

    def draw_callback(self, data):
        self.get_logger().info('Draw graph')
        # draw robot in the middle
        X = [0]
        Y = [0]
        colors = ['black']
        confidence = [1.]

        for cone in self.cones:
            x, y = self.get_euclidean_coordinates(cone[0], cone[1])
            X.append(x)
            Y.append(y)
            confidence.append(cone[2])
            if cone[3] == 0:
                colors.append('blue')
            elif cone[3] == 1:
                colors.append('orange')
            elif cone[3] == 2:
                colors.append('yellow')
            else:
                colors.append('red')

        plt.scatter(X, Y, color=colors, alpha=confidence)
        plt.show()

    @staticmethod
    def get_euclidean_coordinates(angle, distance):
        rad = angle * (np.pi / 180)
        x = distance * np.cos(rad)
        y = distance * np.sin(rad)
        return x, y

    def received_labels(self, data):
        received = np.asarray(data.data)
        received = received.reshape(int(len(received) / 6), 6)
        # x1, y1, x2, y2, conf, label

        received_cones = []
        for cone in received:
            start, end = self.calculate_lidar_range(cone)
            cone_distances = self.lidar_data[round(start):round(end)]
            if len(cone_distances) < 3:
                pass
            else:
                distance = np.median(cone_distances)
                angle = (start + end) / 2.
                # angle, distance, conf, label
                received_cones.append([angle, distance, cone[4], cone[5]])
        self.cones = received_cones

    def received_lidar_data(self, data):
        self.lidar_data = np.asarray(data.data)

    @staticmethod
    def calculate_lidar_range(cone):
        fov = 62.2
        n_pixels = 640
        center = 180
        left = center - (fov / 2)
        deg_per_pixel = fov / n_pixels
        x1 = cone[0]
        x2 = cone[2]
        start = left + (x1 * deg_per_pixel)
        end = left + (x2 * deg_per_pixel)

        return start, end


def main(args=None):
    rclpy.init(args=args)
    cone_loc_node = ConeLocalizationNode()
    rclpy.spin(cone_loc_node)
    cone_loc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
