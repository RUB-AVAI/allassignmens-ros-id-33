import math
import time

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
from avai_messages.msg import Cones, Track
from sklearn.cluster import DBSCAN
import tf_transformations


# cone_color_code: blue = 0, orange = 1, yellow = 2

class ConeLocalizationNode(Node):

    def __init__(self):
        super().__init__('cone_localization_node')

        self.cones_new = []
        self.cones_clustered = []
        self.lidar_data = []
        self.track = []
        self.startup_position = []
        self.calibrated_position = None
        self.relative_position = []
        self.sign = -1  # quick fix for reversed odometry

        self.count_for_DBSCAN = 0
        self.RATE_OF_DBSCAN = 8

        self.fig, self.ax = plt.subplots()

        self.laserfilt = message_filters.Subscriber(self, LaserScan, '/scan', qos_profile=qos_profile_sensor_data)
        self.odomfilt = message_filters.Subscriber(self, Odometry, '/odom')
        self.labelfilt = message_filters.Subscriber(self, Cones, '/images/labels')

        self.mocksub = self.create_subscription(Bool, '/lidar/graph', self.mock_callback, 10)

        self.draw_synchronizer = message_filters.ApproximateTimeSynchronizer([self.laserfilt, self.odomfilt],
                                                                             queue_size=25, slop=.2)
        self.draw_synchronizer.registerCallback(self.synchronized_callback)

        self.cone_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.laserfilt, self.odomfilt, self.labelfilt], queue_size=50, slop=.05)
        self.cone_synchronizer.registerCallback(self.fusion_callback)

    def mock_callback(self, data):
        self.relative_position = [0, 0, -90]
        self.cones_clustered = [[0, 0, 1, 1], [0.2, 0, 1, 1], [0.2, 0.2, 1, 2], [0.2, 0.55, 1, 2], [0.3, 0.8, 1, 2],
                                [0.4, 1.2, 1, 2], [0, 0.2, 0, 0], [0, 0.45, 1, 0], [0.1, 0.8, 1, 0], [0.2, 1.1, 1, 0]]
        cone_knowledge = [[], [], []]
        for cone_known in self.cones_clustered:
            cone_knowledge[int(cone_known[3])].append(cone_known)
        self.track = self.calculate_track(cone_knowledge)
        self.draw()

    def fusion_callback(self, laser, odom, labels):

        self.get_logger().info("Start sensor fusion")
        lidar_data = np.asarray(laser.ranges[::-1])
        r, p, y = tf_transformations.euler_from_quaternion(
            [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
             odom.pose.pose.orientation.w])

        position = np.asarray([odom.pose.pose.position.x, odom.pose.pose.position.y, np.rad2deg(-y)])
        if len(self.startup_position) == 0:
            self.startup_position = position
        if self.calibrated_position is None:
            self.calibrated_position = position

        rel_pos = [self.sign * (position[0] - self.startup_position[0]),
                   self.sign * (position[1] - self.startup_position[1]),
                   (self.startup_position[2] - position[2]) % 360]
        self.relative_position = rel_pos
        cone_labels = labels.cones

        # Checking condition for calibration.
        # If last calibrated position is further away then 0.5 meter, then calibrate
        position_delta = self.get_euclidean_distance(
                cone1=[self.calibrated_position[0], self.calibrated_position[1]],
                cone2=[position[0],position[1]])

        if position_delta >= 0.5 and len(self.cones_clustered) > 0:
            self.calibrate_position(labels=labels, lidar_data=laser, position=rel_pos)

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

                cone_knowledge = [[], [], []]
                for cone_known in self.cones_clustered:
                    cone_knowledge[int(cone_known[3])].append(cone_known)
                self.track = self.calculate_track(cone_knowledge)
            self.count_for_DBSCAN = 0  # To prevent overflow

    def calculate_track(self, cone_data):

        # calculate the trackbeginning from orange cone to yellow/blue
        blue_cones = cone_data[0].copy()
        orange_cones = cone_data[1].copy()
        yellow_cones = cone_data[2].copy()

        if len(orange_cones) == 2 and len(blue_cones) > 0 and len(yellow_cones) > 0:
            first_is_blue = False
            blue_near, blue_distance = self.get_nearest_cone(orange_cones[0], blue_cones)
            yellow_near, yellow_distance = self.get_nearest_cone(orange_cones[0], yellow_cones)
            first_is_blue = blue_distance < yellow_distance

            # wenn first_is_blue, dann ist orange_cone[0] auf der blauen Seite, sonst gelben Seite

            blue_track = []
            yellow_track = []

            if first_is_blue:
                blue_track.append(orange_cones[0])
                blue_track.append(blue_near)
                blue_cones.remove(blue_near)
                yellow_track.append(orange_cones[1])
                y0, y_dist = self.get_nearest_cone(orange_cones[1], yellow_cones)
                yellow_track.append(y0)
                yellow_cones.remove(y0)

            else:
                yellow_track.append(orange_cones[0])
                yellow_track.append(yellow_near)
                yellow_cones.remove(yellow_near)
                blue_track.append(orange_cones[1])
                b0, b_dist = self.get_nearest_cone(orange_cones[1], blue_cones)
                blue_track.append(b0)
                blue_cones.remove(b0)

            blue_track_index = 1
            while len(blue_cones) > 0:
                cone_blue, cone_distance = self.get_nearest_cone(blue_track[blue_track_index], blue_cones)
                blue_track.append(cone_blue)
                blue_cones.remove(cone_blue)
                blue_track_index += 1

            yellow_track_index = 1
            while len(yellow_cones) > 0:
                cone_yellow, cone_distance = self.get_nearest_cone(yellow_track[yellow_track_index], yellow_cones)
                yellow_track.append(cone_yellow)
                yellow_cones.remove(cone_yellow)
                yellow_track_index += 1

            return [blue_track, yellow_track]

        else:
            return []

    def get_nearest_cone(self, reference_cone, cone_list):
        if len(cone_list) == 0:
            return None
        nearest_cone = cone_list[0]
        closest_distance = self.get_euclidean_distance(reference_cone, nearest_cone)
        for listed_cone in cone_list:
            distance = self.get_euclidean_distance(reference_cone, listed_cone)
            if distance < closest_distance:
                closest_distance = distance
                nearest_cone = listed_cone
        return nearest_cone, closest_distance

    def use_dbscan(self, data_set, _min_samples=4, _eps=.1):
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
                clustered_cones[int(elem[4])].append(cone_tupel)

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
        rel_pos.append(self.sign * (position[0] - self.startup_position[0]))
        rel_pos.append(self.sign * (position[1] - self.startup_position[1]))
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
        self.ax.set_ylim(-2 + +Y[0], 2 + Y[0])
        self.ax.set_xlim(-2 + X[0], 2 + X[0])
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
        x_cone_clustered = []
        y_cone_clustered = []
        colors_clustered = []
        for cone in self.cones_clustered:
            x, y = cone[0], cone[1]
            x_cone_clustered.append(x)
            y_cone_clustered.append(y)

            if cone[3] == 0:
                colors_clustered.append('blue')
            elif cone[3] == 1:
                colors_clustered.append('orange')
            elif cone[3] == 2:
                colors_clustered.append('yellow')
            else:
                colors_clustered.append('red')  # error case

        x_cone = []
        y_cone = []
        colors = []
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
        self.ax.scatter(x_cone, y_cone, color=colors, alpha=.3)
        self.ax.scatter(x_cone_clustered, y_cone_clustered, color=colors_clustered, marker="x")

        if len(self.track) == 2:
            blue_track = self.track[0]
            yellow_track = self.track[1]
            blue_x = [cone[0] for cone in blue_track]
            blue_y = [cone[1] for cone in blue_track]
            yellow_x = [cone[0] for cone in yellow_track]
            yellow_y = [cone[1] for cone in yellow_track]
            self.ax.plot(blue_x, blue_y, color='blue')
            self.ax.plot(yellow_x, yellow_y, color='yellow')
        plt.pause(.1)

    @staticmethod
    def get_euclidean_distance(cone1, cone2):
        x1 = cone1[0]
        y1 = cone1[1]
        x2 = cone2[0]
        y2 = cone2[1]
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

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


    @staticmethod
    def calculate_range_and_angle(cone: tuple, robot: tuple) -> tuple:
        range = ConeLocalizationNode.get_euclidean_distance(cone, robot)
        angle = math.atan((cone[1] - robot[1]) / (cone[0] - robot[0]))
        return range, angle

    def calibrate_position(self, labels, lidar_data, position):
        for cone in self.cones_clustered:
            rangee, angle = self.calculate_range_and_angle(cone, position)


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
                #print("x: ", x, "y: ", y)
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
