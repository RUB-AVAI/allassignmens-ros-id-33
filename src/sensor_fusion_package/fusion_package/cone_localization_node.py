import cv2
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
import tf_transformations


class ConeLocalizationNode(Node):

    def __init__(self):
        super().__init__('cone_localization_node')

        self.cones_new = []
        self.cones_clustered = []
        self.lidar_data = []
        self.startup_position = []
        self.relative_position = []

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
            [self.laserfilt, self.odomfilt, self.labelfilt], queue_size=1000, slop=.1)
        self.cone_synchronizer.registerCallback(self.fusion_callback)

    def fusion_callback(self, laser, odom, labels):

        self.get_logger().info("Start sensor fusion")
        lidar_data = np.asarray(laser.ranges[::-1])
        r, p, y = tf_transformations.euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])

        position = np.asarray([odom.pose.pose.position.x, odom.pose.pose.position.y, np.rad2deg(-y)])
        if len(self.startup_position) == 0:
            self.startup_position = position

        rel_pos = [position[0] - self.startup_position[0], position[1] - self.startup_position[1],
                   (self.startup_position[2] - position[2]) % 360]
        self.relative_position = rel_pos
        cone_labels = labels.cones

        cones = self.received_labels(cone_labels, lidar_data, rel_pos)
        for cone in cones:
            self.cones_new.append(cone)

        self.count_for_DBSCAN += 1

        if self.count_for_DBSCAN == self.RATE_OF_DBSCAN:
            self.get_logger().info("using DBSCAN")
            # cluster new cones and append them to cones_clustered

            if len(self.cones_new) > 0:
                clustered_cones = self.use_dbscan(self.cones_new)
                for c in clustered_cones:
                    self.cones_clustered.append(c)
                self.cones_new = []

            # cluster already clustered cones
            if len(self.cones_clustered) > 0:
                self.cones_clustered = self.use_dbscan(self.cones_clustered, 1)
                pass
            self.count_for_DBSCAN = 0  # To prevent overflow

    def use_dbscan(self, data_set, _min_samples=2, _eps=.14):
        # clustering over x, y, color
        # TODO: Check whether values are saved correctly for clustering!
        x_train = []
        # We set the z coordinate of the cones points in DBSCAN as DIGIT_FOR_COLOR * (_eps + 1)
        # to prevent clustering cones of different color to one.
        # See report of assignment 7 for detailed explanation.
        for c in data_set:
            x_train.append([c[0], c[1], c[3] * (_eps + 1)])

        dbscan = DBSCAN(eps=_eps, min_samples=_min_samples).fit(x_train)

        cluster_labels = dbscan.labels_

        # in cluster_lables sind die entsprechenden Labels für jeden Eintrag --> neuer Wert --> neues Hütchen

        DBSCAN_dataset = data_set.copy()
        # using np.concatenate method, because faster
        # TODO: Check concatenation!
        DBSCAN_dataset = np.concatenate((DBSCAN_dataset, cluster_labels[:, np.newaxis]), axis=1)

        amount_cones = len(set(cluster_labels))
        clustered_cones = [None] * amount_cones

        for i in range(len(clustered_cones)):
            clustered_cones[i] = []

        for elem in DBSCAN_dataset:
            if elem[4] != -1:
                cone_tupel = [elem[0], elem[1], elem[2], elem[3]]


        new_cone_representation = []
        for cluster in clustered_cones:
            if len(cluster) > 0:
                x_coordinates = [row[0] for row in cluster]
                y_coordinates = [row[1] for row in cluster]
                x_mean = np.mean(x_coordinates)
                y_mean = np.mean(y_coordinates)
                cone_color = cluster[0][3]
                new_cone_representation.append([x_mean, y_mean, 1, cone_color])
        return new_cone_representation

    def synchronized_callback(self, laser, odom):
        lidar_data = np.asarray(laser.ranges[::-1])
        r, p, y = tf_transformations.euler_from_quaternion(
            [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
             odom.pose.pose.orientation.w])
        position = [odom.pose.pose.position.x, odom.pose.pose.position.y, np.rad2deg(-y)]

        if len(self.startup_position) == 0:
            self.startup_position = position

        rel_pos = []
        rel_pos.append(position[0] - self.startup_position[0])
        rel_pos.append(position[1] - self.startup_position[1])
        rel_pos.append((self.startup_position[2] - position[2]) % 360)
        self.relative_position = rel_pos
        l_data = []
        for angle in range(len(lidar_data)):
            distance = lidar_data[angle]

            x, y = self.get_euclidean_coordinates((angle - rel_pos[2]) % 360, distance)

            x += self.relative_position[0]
            y += self.relative_position[1]
            l_data.append([x, y])

        self.lidar_data = l_data
        self.draw()

    def draw(self):
        """
        Plots the current situation with matplotlib
        """
        # draw robot in the middle
        self.ax.cla()
        X = [self.relative_position[0]]
        Y = [self.relative_position[1]]
        self.ax.scatter(self.relative_position[0], self.relative_position[1], color='black', alpha=1)

        lidar_x = []
        lidar_y = []

        for entry in self.lidar_data:
            lidar_x.append(entry[0])
            lidar_y.append(entry[1])

        self.ax.scatter(lidar_x, lidar_y, s=.4)
        self.ax.set_ylim(-2, 2)
        self.ax.set_xlim(-2, 2)
        line_x = []
        line_y = []
        dist = np.linspace(0, 2, 100)

        orientation = self.relative_position[2]

        for d in dist:
            lx, ly = self.get_euclidean_coordinates((148 - orientation) % 360, d)
            line_x.append(lx + X)
            line_y.append(ly + Y)

        for d in dist:
            lx, ly = self.get_euclidean_coordinates((212 - orientation) % 360, d)
            line_x.append(lx + X)
            line_y.append(ly + Y)

        self.ax.plot(line_x, line_y, color='green')
        x_cone = []
        y_cone = []
        colors = []
        for cone in self.cones_clustered:
            x, y = cone[0], cone[1]
            x_cone.append(x)
            y_cone.append(y)

            if cone[3] == 0:
                colors.append('blue')
            elif cone[3] == 1:
                colors.append('orange')
            elif cone[3] == 2:
                colors.append('yellow')
            else:
                colors.append('red')  # error case

        for cone in self.cones_new:
            x, y = cone[0], cone[1]
            x_cone.append(x)
            y_cone.append(y)

            if cone[3] == 0:
                colors.append('blue')
            elif cone[3] == 1:
                colors.append('orange')
            elif cone[3] == 2:
                colors.append('yellow')
            else:
                colors.append('red')  # error case
        self.ax.scatter(x_cone, y_cone, color=colors)
        plt.pause(.5)

    @staticmethod
    def get_euclidean_coordinates(angle, distance) -> tuple:
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

    def received_labels(self, labels, lidar_data, position) -> list:
        # could be static method
        """
        Returns array of the cones for the received labels in the following form:
        [x, y, confidence, label]

        Note that x and y are the absolute position in the room
        """
        labels = np.asarray(labels).reshape(int(len(labels) / 6), 6)
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

            if len(cone_distances) < 2:
                continue
            else:
                distance = np.median(cone_distances)
                angle = (start + end) / 2.
                x, y = self.get_euclidean_coordinates((angle - position[2]) % 360, distance)
                delta_rotation = position[2]
                x += position[0]
                y += position[1]
                print("x: ", x, "y: ", y)
                received_cones.append([x, y, cone[4], cone[5]])
        return received_cones

    @staticmethod
    def calculate_lidar_range(cone) -> tuple:
        fov = 64
        left = 180 - (fov / 2)
        n_pixels = 640
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
