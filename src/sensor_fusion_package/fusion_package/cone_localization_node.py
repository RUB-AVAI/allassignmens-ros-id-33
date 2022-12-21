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
        self.labelsub = self.create_subscription(Float32MultiArray, '/images/labels', self.received_labels, 10)
        self.lasersub = self.create_subscription(Float32MultiArray, '/laser/scanned', self.received_lidar_data, 10)
        self.graphsub = self.create_subscription(Bool, '/lidar/graph', self.draw_callback, 10)

    def draw_callback(self, data):
        self.get_logger().info('Draw graph')
        # draw robot in the middle
        X = [0]
        Y = [0]
        colors = ['black']
        confidence = [1.]

        lidar_x = []
        lidar_y = []
        for angle, distance in enumerate(self.lidar_data):
            x, y = self.get_euclidean_coordinates(angle, distance)
            lidar_x.append(x)
            lidar_y.append(y)

        plt.scatter(lidar_x, lidar_y, s=.4)


        for cone in self.cones:
            dist = np.linspace(0, 2, 100)
            line_x = []
            line_y = []
            for d in dist:
                lx, ly = self.get_euclidean_coordinates(cone[0], d)
                line_x.append(lx)
                line_y.append(ly)
            plt.plot(line_x, line_y)

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
        x = distance * np.cos(-rad)
        y = distance * np.sin(-rad)
        return x, y

    def received_labels(self, data):
        received = np.asarray(data.data)
        received = received.reshape(int(len(received) / 6), 6)
        # x1, y1, x2, y2, conf, label

        received_cones = []
        for cone in received:
            start, end = self.calculate_lidar_range(cone)
            difference = end - start
            offset = 3.5
            start += offset
            end += offset
            shift = difference*1/6

            cone_distances = self.lidar_data[round(start+shift):round(end-shift)]
            self.get_logger().info(f"The cone with color {cone[5]} started with {cone[0]} and ended with {cone[2]}")
            print(cone_distances)
            if len(cone_distances) < 3:
                pass
            else:
                distance = np.median(cone_distances)
                angle = (start + end) / 2.
                # angle, distance, conf, label
                received_cones.append([angle, distance, cone[4], cone[5]])
        self.cones = received_cones

    def received_lidar_data(self, data):
        self.lidar_data = np.asarray(data.data[::-1])

    @staticmethod
    def calculate_lidar_range(cone):
        fov = 62.2
        left  = 180 - (fov / 2)
        n_pixels = 640
        print("cone 0: ", cone[0])
        print("cone 2:", cone[2])
        start = left + cone[0] * fov/n_pixels
        end = left + cone[2] * fov/n_pixels

        return start, end


def main(args=None):
    rclpy.init(args=args)
    cone_loc_node = ConeLocalizationNode()
    rclpy.spin(cone_loc_node)
    cone_loc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
