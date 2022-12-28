import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
from sensor_msgs.msg import LaserScan
from matplotlib import pyplot as plt
from rclpy.qos import qos_profile_sensor_data, QoSProfile


class ConeLocalizationNode(Node):


    def __init__(self):
        super().__init__('cone_localization_node')

        self.cones = []
        self.lidar_cache = []
        self.lidar_cache_size = 0
        self.lidar_cache_target = 100
        self.lidar_cache_current = -1
        self.labelsub = self.create_subscription(Float32MultiArray, '/images/labels', self.received_labels, 10)
        self.lasersub = self.create_subscription(LaserScan, '/scan', self.received_lidar_data, qos_profile=qos_profile_sensor_data)
        self.graphsub = self.create_subscription(Bool, '/lidar/graph', self.draw_callback, 10)
        self.fig, self.ax = plt.subplots()

    def draw_callback(self, data):
        self.get_logger().info('Draw graph')
        # draw robot in the middle
        self.ax.cla()
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

        self.ax.scatter(lidar_x, lidar_y, s=.4)

        line_x = []
        line_y = []
        dist = np.linspace(0, 2, 100)
        for d in dist:
            lx, ly = self.get_euclidean_coordinates(148, d)
            line_x.append(lx)
            line_y.append(ly)
        for d in dist:
            lx, ly = self.get_euclidean_coordinates(212, d)
            line_x.append(lx)
            line_y.append(ly)
        self.ax.plot(line_x, line_y)

        for cone in self.cones:
            x, y = self.get_euclidean_coordinates(cone[0], cone[1])
            print("distance: ", cone[1], "labelclass: ", cone[3], "angle: ", cone[0])
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

        self.ax.scatter(X, Y, color=colors, alpha=confidence)
        plt.pause(.1)

    @staticmethod
    def get_euclidean_coordinates(angle, distance):
        rad = angle * (np.pi / 180)
        x = distance * np.cos(rad)
        y = distance * np.sin(-rad)
        return x, y

    def received_labels(self, data):
        self.draw_callback(None)
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
            shift = difference * 1 / 6

            cone_distances = self.lidar_data[round(start + shift):round(end - shift)]
            cone_distances = list(filter(lambda x: 0 < x < 2.5, cone_distances))

            self.get_logger().info(f"The cone with color {cone[5]} started with {cone[0]} and ended with {cone[2]}")
            print(cone_distances)
            if len(cone_distances) < 2:
                continue
            else:
                distance = np.median(cone_distances)
                angle = (start + end) / 2.
                # angle, distance, conf, label
                received_cones.append([angle, distance, cone[4], cone[5]])
        self.cones = received_cones

    def received_lidar_data(self, data):

        lidar_data = np.asarray(data.ranges[::-1])

        if self.lidar_cache_size < self.lidar_cache_target:
            self.lidar_cache.append(lidar_data)
            self.lidar_cache_size += 1
            self.lidar_cache_current += 1



    @staticmethod
    def calculate_lidar_range(cone):
        fov = 64
        left = 180 - (fov / 2)
        n_pixels = 640
        print("cone 0: ", cone[0])
        print("cone 2:", cone[2])
        start = left + cone[0] * fov / n_pixels
        end = left + cone[2] * fov / n_pixels

        return start, end


def main(args=None):
    rclpy.init(args=args)
    qos = QoSProfile(depth=10)
    cone_loc_node = ConeLocalizationNode()
    rclpy.spin(cone_loc_node)
    cone_loc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
