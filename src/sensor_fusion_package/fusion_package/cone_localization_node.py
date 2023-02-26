import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from std_msgs.msg import Bool, Float64MultiArray, Float64
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from avai_messages.msg import Cones, Track

import message_filters
import tf_transformations
import numpy as np
from sklearn.cluster import DBSCAN
from matplotlib import pyplot as plt
from typing import List


# cone_color_code: blue = 0, orange = 1, yellow = 2

class ConeLocalizationNode(Node):

    def __init__(self):
        super().__init__('cone_localization_node')

        # variables for conedetection
        self.cones_new = []
        self.cones_clustered = []
        self.lidar_data = []

        # variables for track calculation
        self.track = None
        self.autonomous_track = None
        self.calibrated_position = []
        self.position = []

        # variables for autonomous drive
        self.start = []
        self.start_arrived = False
        self.next_drive_commands = []
        self.robot_state = 0
        self.robot_state_list = ["detecting_cones", "driving", "calibrating_position", "rotating_for_information_right",
                                 "detecting_after_rotation", "rotating_for_information_left"]

        # variables for DBSCAN
        self.count_for_DBSCAN = 0
        self.RATE_OF_DBSCAN = 12

        # constants
        self.avarage_radius = .002
        self.calibration_bias = .3

        # variables for plotting
        self.fig, self.ax = plt.subplots()

        # synchronized subscriptions
        self.laserfilt = message_filters.Subscriber(self, LaserScan, '/scan', qos_profile=qos_profile_sensor_data)
        self.odomfilt = message_filters.Subscriber(self, Odometry, '/codom')
        self.labelfilt = message_filters.Subscriber(self, Cones, '/images/labels')
        self.draw_synchronizer = message_filters.ApproximateTimeSynchronizer([self.laserfilt, self.odomfilt],
                                                                             queue_size=500, slop=.05)
        self.draw_synchronizer.registerCallback(self.synchronized_callback)
        self.cone_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.laserfilt, self.odomfilt, self.labelfilt], queue_size=500, slop=.05)
        self.cone_synchronizer.registerCallback(self.fusion_callback)

        # subscriber and publisher for driving mission
        self.trackpublisher = self.create_publisher(Track, '/track', 10)
        self.drivesub = self.create_subscription(Bool, 'drive', self.drive_callback, 10)
        self.rotation_publisher = self.create_publisher(Float64, '/rotation', 10)

        self.calbiration_publisher = self.create_publisher(Float64MultiArray, '/position_calibration', 10)

    def drive_callback(self, data):
        print("drive_callback")
        if data.data:
            self.robot_state = 2  # calibrate_rotation
        else:
            self.robot_state = (self.robot_state + 1) % len(self.robot_state_list)

    def fusion_callback(self, laser, odom, labels):

        print(self.robot_state_list[self.robot_state], ": ", self.robot_state)
        if self.robot_state == 1:  # if == 0 than robot should detect cones
            return

        elif self.robot_state == 2:  # calibration of position
            offset_x, offset_y = self.recalibrate_position(laser)
            calibration_message = Float64MultiArray()
            calibration_message.data = [offset_x * self.calibration_bias, offset_y * self.calibration_bias]
            print(offset_x, offset_y)
            self.calbiration_publisher.publish(calibration_message)
            self.robot_state = 0
            return
        elif self.robot_state == 3 or self.robot_state == 5:
            return

        lidar_data = np.asarray(laser.ranges[::-1])
        r, p, y = tf_transformations.euler_from_quaternion(
            [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
             odom.pose.pose.orientation.w])

        self.position = [odom.pose.pose.position.x, odom.pose.pose.position.y, np.rad2deg(y)]

        cone_labels = labels.cones
        cones = self.received_labels(cone_labels, lidar_data, self.position)

        for cone in cones:
            self.cones_new.append(cone)

        self.count_for_DBSCAN += 1

        if (self.count_for_DBSCAN % self.RATE_OF_DBSCAN) == 0:
            self.get_logger().info("using DBSCAN")

            # cluster new cones and append them to cones_clustered

            if len(self.cones_new) > 0:
                clustered_cones = self.use_dbscan(self.cones_new, _min_samples=self.RATE_OF_DBSCAN - 4, _eps=0.15)
                for c in clustered_cones:
                    self.cones_clustered.append(c)
                self.cones_new = []
            # cluster already clustered cones
            if len(self.cones_clustered) > 0:
                self.cones_clustered = self.use_dbscan(self.cones_clustered, _min_samples=1, _eps=0.15)

                cone_knowledge = [[], [], []]
                for cone_known in self.cones_clustered:
                    cone_knowledge[int(cone_known[3])].append(cone_known)

                if self.robot_state == 0:

                    self.track = self.calculate_track(cone_knowledge)
                    self.autonomous_track = self.calculate_drive(self.track)

                    if self.start_arrived: # wir waren bereits am Start, also an den orangen Cones
                        self.next_drive_commands = self.calculate_next_drive_commands(self.autonomous_track,
                                                                                      self.position)
                    else:
                        self.next_drive_commands = self.start
                        self.start_arrived = True

                    if self.next_drive_commands is None:
                        # wir müssen uns rotieren, um Informationen zu sammeln
                        self.robot_state = 3
                        target_angle = np.deg2rad(self.position[2] - 40) + np.pi
                        msg = Float64()
                        msg.data = target_angle
                        self.rotation_publisher.publish(msg)
                        return

                    else:

                        # publish the track_message
                        track_message = Track()
                        track_message.start = odom
                        drive_x = [checkpoint[0] for checkpoint in self.next_drive_commands]
                        drive_y = [checkpoint[1] for checkpoint in self.next_drive_commands]
                        track_message.x = drive_x
                        track_message.y = drive_y

                        if len(track_message.x) == 0:
                            # wir müssen uns rotieren, um Informationen zu sammeln
                            self.robot_state = 3
                            print(self.position[2])
                            target_angle = np.deg2rad(self.position[2] - 40) + np.pi
                            msg = Float64()
                            msg.data = target_angle
                            self.rotation_publisher.publish(msg)
                            self.count_for_DBSCAN = 0

                        else:
                            self.trackpublisher.publish(track_message)
                            self.robot_state = 1

                elif self.robot_state == 4:
                    self.robot_state = 5
                    target_angle = np.deg2rad(self.position[2] + 80) + np.pi
                    msg = Float64()
                    msg.data = target_angle
                    self.rotation_publisher.publish(msg)

            self.count_for_DBSCAN = 0  # To prevent overflow

    def recalibrate_position(self, laser):

        # PARAMS FOR FUNCTION
        eps_for_DBSCAN = .01
        min_samples_for_DBSCAN = 4
        max_distance = .6
        distance_threshold = 0.2

        offset_x_list = []
        offset_y_list = []
        laser = laser.ranges[::-1]  # swapped lidar_data
        modified_laser = []
        for i in range(len(laser)):
            x, y = self.get_euclidean_coordinates((i - self.position[2]) % 360, laser[i])
            x += self.position[0]
            y += self.position[1]
            modified_laser.append([x, y, 1, 1])
        clustered_lidar_data = self.use_dbscan(modified_laser, _min_samples=min_samples_for_DBSCAN,
                                               _eps=eps_for_DBSCAN)  # clustering the lidar_data, hyperparameters might need to be changed
        cone_knowledge = self.cones_clustered.copy()
        # same max distance for knowledge base
        for cone in cone_knowledge:
            cone_dist = self.get_euclidean_distance(cone, self.position)
            if cone_dist > max_distance:
                cone_knowledge.remove(cone)  # now only cones with max dist of 1 are left

        # ploting for debuggin
        self.ax.cla()
        self.ax.scatter(self.position[0], self.position[1], color='black', alpha=1)

        self.ax.set_ylim(-2 + self.position[1], 2 + self.position[1])
        self.ax.set_xlim(-2 + self.position[0], 2 + self.position[0])

        cone_x = []
        cone_y = []
        cone_color = []
        for cone in clustered_lidar_data:
            cone_x.append(cone[0])
            cone_y.append(cone[1])
            if cone[3] == 0:
                cone_color.append('blue')
            elif cone[3] == 1:
                cone_color.append('orange')
            elif cone[3] == 2:
                cone_color.append('yellow')
            else:
                cone_color.append('red')  # error case

        self.ax.scatter(cone_x, cone_y, color=cone_color)

        lidar_x = []
        lidar_y = []

        for lidar_point in modified_laser:
            lidar_x.append(lidar_point[0])
            lidar_y.append(lidar_point[1])

        self.ax.scatter(lidar_x, lidar_y, s=.2)
        plt.pause(.1)

        # matching cones and lidar data
        for cone in cone_knowledge:
            matched_cluster, distance = self.get_nearest_cone(cone, clustered_lidar_data)
            if distance < distance_threshold:
                offset_x_list.append(matched_cluster[0] - cone[0])
                offset_y_list.append(matched_cluster[1] - cone[1])
        if len(offset_x_list) < 1:
            return .0, .0
        # return the mean of the distances in x, y and angle as the offset
        return np.mean(offset_x_list), np.mean(offset_y_list)

    def calculate_track(self, cone_data):

        # params
        MAX_DIST = .6

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
                if cone_distance < MAX_DIST:
                    blue_track.append(cone_blue)
                    blue_cones.remove(cone_blue)
                    blue_track_index += 1
                else:
                    break

            yellow_track_index = 1
            while len(yellow_cones) > 0:
                cone_yellow, cone_distance = self.get_nearest_cone(yellow_track[yellow_track_index], yellow_cones)
                if cone_distance < MAX_DIST:
                    yellow_track.append(cone_yellow)
                    yellow_cones.remove(cone_yellow)
                    yellow_track_index += 1
                else:
                    break

            return [blue_track, yellow_track]
        else:
            return None

    def calculate_drive(self, track):

        # PARAMS
        dist_threshold = .8

        if len(track) == 2:
            blue_track = track[0].copy()
            yellow_track = track[1].copy()
            out = []
            for b_cone in blue_track:
                y_cone, y_dist = self.get_nearest_cone(b_cone, yellow_track)
                if y_dist < dist_threshold:
                    b_x = b_cone[0]
                    b_y = b_cone[1]
                    y_x = y_cone[0]
                    y_y = y_cone[1]
                    x_mid = (b_x + y_x) / 2
                    y_mid = (b_y + y_y) / 2
                    out.append([x_mid, y_mid])

            if len(out) > 0:
                self.start.append(out[0])
            return out
        else:
            return []

    def calculate_next_drive_commands(self, auto_track, position):

        robot_position_as_cone = [position[0], position[1], 1, 1]

        nearest_checkpoint, distance_to_checkpoint = self.get_nearest_cone(robot_position_as_cone, auto_track)

        index_of_current_checkpoint = auto_track.index(nearest_checkpoint)
        next_checkpoint = index_of_current_checkpoint + 1

        if next_checkpoint > len(auto_track) - 1:
            return None
        return [auto_track[next_checkpoint]]

    def get_nearest_cone(self, reference_cone: list, cone_list: List[list]):

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

        position = [odom.pose.pose.position.x, odom.pose.pose.position.y, np.rad2deg(y)]

        self.position = [position[0], position[1], position[2]]
        l_data = []
        for angle in range(len(lidar_data)):
            distance = lidar_data[angle]

            x, y = self.get_euclidean_coordinates((angle - position[2]) % 360, distance)

            x += self.position[0]
            y += self.position[1]
            l_data.append([x, y])

        self.lidar_data = l_data
        self.draw()

    def draw(self):
        """
        Plots the current situation with matplotlib
        """

        #plotsize
        PLOT_SIZE = 3.5

        # draw robot in the middle
        self.ax.cla()
        X = [self.position[0]]
        Y = [self.position[1]]
        self.ax.scatter(self.position[0], self.position[1], color='black', alpha=1)

        lidar_x = []
        lidar_y = []

        for entry in self.lidar_data:
            lidar_x.append(entry[0])
            lidar_y.append(entry[1])

        self.ax.scatter(lidar_x, lidar_y, s=.4)
        self.ax.set_ylim(-PLOT_SIZE + +Y[0], PLOT_SIZE + Y[0])
        self.ax.set_xlim(-PLOT_SIZE + X[0], PLOT_SIZE + X[0])
        line_x = []
        line_y = []
        dist = np.linspace(0, 2, 100)

        orientation = self.position[2]

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

        if self.track is not None:
            blue_track = self.track[0]
            yellow_track = self.track[1]
            blue_x = [cone[0] for cone in blue_track]
            blue_y = [cone[1] for cone in blue_track]
            yellow_x = [cone[0] for cone in yellow_track]
            yellow_y = [cone[1] for cone in yellow_track]
            self.ax.plot(blue_x, blue_y, color='blue')
            self.ax.plot(yellow_x, yellow_y, color='yellow')

        if self.next_drive_commands is not None:
            if len(self.next_drive_commands) > 0:
                self.ax.scatter(self.next_drive_commands[0][0], self.next_drive_commands[0][1], color='tab:gray')

        plt.pause(.1)  # normal code when working with the bot

    @staticmethod
    def get_euclidean_distance(cone1, cone2):
        x1 = cone1[0]
        y1 = cone1[1]
        x2 = cone2[0]
        y2 = cone2[1]
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    @staticmethod
    def get_angle(current_position, cone):
        normalized_position_x = cone[0] - current_position[0]
        normalized_position_y = cone[1] - current_position[1]

        return np.arctan2(normalized_position_y, normalized_position_x) + cone[2]

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
            cone_distances = list(filter(lambda x: 0 < x < 1.7, cone_distances))

            if len(cone_distances) < 2:
                continue
            else:
                distance = np.median(cone_distances)
                angle = (start + end) / 2.
                x, y = self.get_euclidean_coordinates((angle - position[2]) % 360, distance + self.avarage_radius)
                x += position[0]
                y += position[1]
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
