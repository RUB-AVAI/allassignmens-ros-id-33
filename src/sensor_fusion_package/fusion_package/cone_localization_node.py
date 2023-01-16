import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt
from rclpy.qos import qos_profile_sensor_data, QoSProfile
import message_filters
from avai_messages.msg import Cones
from sklearn.cluster import DBSCAN


class ConeLocalizationNode(Node):

    def __init__(self):
        super().__init__('cone_localization_node')

        # TODO: also cache labels and odometry data with timestamps

        self.cones = []
        self.lidar_data = []
        self.position = []

        self.count_for_DBSCAN = 0
        self.RATE_OF_DBSCAN = 5

        self.fig, self.ax = plt.subplots()

        self.laserfilt = message_filters.Subscriber(self, LaserScan, '/scan', qos_profile=qos_profile_sensor_data)
        self.odomfilt = message_filters.Subscriber(self, Odometry, '/odom')
        self.labelfilt = message_filters.Subscriber(self, Cones, '/images/labels')

        self.draw_synchronizer = message_filters.ApproximateTimeSynchronizer([self.laserfilt, self.odomfilt],
                                                                             queue_size=25, slop=.2)
        self.draw_synchronizer.registerCallback(self.synchronized_callback)

        self.cone_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.laserfilt, self.odomfilt, self.labelfilt], queue_size=1000, slop=.5)
        self.cone_synchronizer.registerCallback(self.fusion_callback)

    def fusion_callback(self, laser, odom, labels):

        self.get_logger().info("Start sensor fusion")
        lidar_data = np.asarray(laser.ranges[::-1])
        position = np.asarray([odom.pose.pose.position.x, odom.pose.pose.position.y])
        cone_labels = labels.cones

        cones = self.received_labels(cone_labels, lidar_data, position)
        for cone in cones:
            self.cones.append(cone)

        self.count_for_DBSCAN += 1
        if self.count_for_DBSCAN == self.RATE_OF_DBSCAN:
            self.use_dbscan()
            self.count_for_DBSCAN = 0 # To prevent overflow

    def use_dbscan(self):

        #clustering over x, y, color
        x_train = [c[0] + c[1] + c[3] for c in self.cones]


        # variables
        _eps = .5 # max distance to be considered in neighborhood
        _min_samples = 3

        dbscan = DBSCAN(eps=_eps, min_samples=_min_samples).fit(x_train)
        cluster_labels = dbscan.labels_

        # in cluster_lables sind die entsprechenden Labels für jeden Eintrag --> neuer Wert --> neues Hütchen

        DBSCAN_dataset = self.cones.copy()
        DBSCAN_dataset = [DBSCAN_dataset[i] + cluster_labels[i] for i in range(len(DBSCAN_dataset))]

        # every label only once in the set(cluster_labels) to get the amount of different cones we clustered
        amount_cones = len(set(cluster_labels))
        clustered_cones = [None]*amount_cones
        for elem in DBSCAN_dataset:
            clustered_cones[elem[-1] + 1].append(elem[0:3])

        # clustered_cones enthält jetzt für jedes clustered cone eine Liste an Positionen. clustered[0] =  "outliers"
        new_cone_representation = []
        for cone in clustered_cones:
            if len(cone) > 0:
                x_coordinates = [row[0] for row in cone]
                y_coordinates = [row[1] for row in cone]
                x_mean = np.mean(x_coordinates)
                y_mean = np.mean(y_coordinates)
                cone_color = cone[0][3]
                new_cone_representation.append([x_mean, y_mean, 1, cone_color])
        self.cones = new_cone_representation

    def synchronized_callback(self, laser, odom):
        lidar_data = np.asarray(laser.ranges[::-1])
        position = np.asarray([odom.pose.pose.position.x, odom.pose.pose.position.y])

        self.position = position
        for idx, point in enumerate(lidar_data):
            lidar_data[idx] = point + self.position
        self.lidar_data = lidar_data

        self.draw()

    def draw(self):
        """
        Plots the current situation with matplotlib
        """
        # draw robot in the middle
        self.ax.cla()
        X = [self.position[0]]
        Y = [self.position[1]]
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
            line_x.append(X[0] + lx)
            line_y.append(Y[0] + ly)
        for d in dist:
            lx, ly = self.get_euclidean_coordinates(212, d)
            line_x.append(X[0] + lx)
            line_y.append(Y[0] + ly)

        self.ax.plot(line_x, line_y)

        for cone in self.cones:
            x, y = cone[0], cone[1]
            # print("distance: ", cone[1], "labelclass: ", cone[3], "angle: ", cone[0])
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
                colors.append('red') # error case

        self.ax.scatter(X, Y, color=colors, alpha=confidence)
        plt.pause(.1)

    @staticmethod
    def get_euclidean_coordinates(angle, distance):
        """
        Calculates the euclidean coordinates from (0,0) for given angle (in degree) and distance
        :param angle:
        :param distance:
        :return:
        """
        rad = angle * (np.pi / 180)
        x = distance * np.cos(rad)
        y = distance * np.sin(-rad)
        return x, y

    def received_labels(self, labels, lidar_data, position):
        # could be static method
        """
        Returns array of the cones for the received labels in the following form:
        [x, y, confidence, label]

        Note that x and y are the absolute position in the room
        """
        labels = labels.reshape(int(len(labels) / 6), 6)
        # x1, y1, x2, y2, conf, label

        received_cones = []
        for cone in labels:
            start, end = self.calculate_lidar_range(cone)
            difference = end - start
            offset = 3.5
            start += offset
            end += offset
            shift = difference * 1 / 6

            cone_distances = lidar_data[round(start + shift):round(end - shift)]
            cone_distances = list(filter(lambda x: 0 < x < 2.5, cone_distances))

            # self.get_logger().info(f"The cone with color {cone[5]} started with {cone[0]} and ended with {cone[2]}")
            # print(cone_distances)
            if len(cone_distances) < 2:
                continue
            else:
                distance = np.median(cone_distances)
                angle = (start + end) / 2.
                x, y = self.get_euclidean_coordinates(angle, distance)
                x += position[0]
                y += position[1]
                received_cones.append([x, y, cone[4], cone[5]])
        return received_cones

    @staticmethod
    def calculate_lidar_range(cone):
        fov = 64
        left = 180 - (fov / 2)
        n_pixels = 640
        # print("cone 0: ", cone[0])
        # print("cone 2:", cone[2])
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
