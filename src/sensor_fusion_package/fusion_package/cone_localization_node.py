import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from avai_messages.msg import Cones, Track

import message_filters
import tf_transformations
import numpy as np
from sklearn.cluster import DBSCAN
from matplotlib import pyplot as plt
from typing import List
from collections import OrderedDict
import bisect


# cone_color_code: blue = 0, orange = 1, yellow = 2

class ConeLocalizationNode(Node):

    def __init__(self):
        super().__init__('cone_localization_node')

        self.cones_new = []
        self.cones_clustered = []
        self.lidar_data = []
        self.track = []
        self.autonomous_track = []
        self.calibrated_position = []
        self.position = []
        self.start = []
        self.start_arrived = False

        # Last calculated offsets of odometry
        self.last_x_offset, self.last_y_offset = 0, 0

        self.next_drive_commands = []

        self.drive = False

        self.count_for_DBSCAN = 0
        self.RATE_OF_DBSCAN = 8

        self.fig, self.ax = plt.subplots()

        self.laserfilt = message_filters.Subscriber(self, LaserScan, '/scan', qos_profile=qos_profile_sensor_data)
        self.odomfilt = message_filters.Subscriber(self, Odometry, '/codom')
        self.labelfilt = message_filters.Subscriber(self, Cones, '/images/labels')

        self.drivesub = self.create_subscription(Bool, 'drive', self.drive_callback, 10)

        self.trackpublisher = self.create_publisher(Track, '/track', 10)

        self.draw_synchronizer = message_filters.ApproximateTimeSynchronizer([self.laserfilt, self.odomfilt],
                                                                             queue_size=500, slop=.2)
        self.draw_synchronizer.registerCallback(self.synchronized_callback)

        self.cone_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.laserfilt, self.odomfilt, self.labelfilt], queue_size=500, slop=.2)
        self.cone_synchronizer.registerCallback(self.fusion_callback)

    def drive_callback(self, data):
        self.drive = False

    def fusion_callback(self, laser, odom, labels):

        if self.drive:
            self.get_logger().info("nothing")
            return
        self.get_logger().info("fusion callback")

        lidar_data = np.asarray(laser.ranges[::-1])
        r, p, y = tf_transformations.euler_from_quaternion(
            [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
             odom.pose.pose.orientation.w])

        self.position = [-odom.pose.pose.position.x, odom.pose.pose.position.y, np.rad2deg(y)]

        cone_labels = labels.cones


        """        
        # Checking condition for calibration.
        # If last calibrated position is further away then 0.5 meter, then calibrate
        position_delta = self.get_euclidean_distance(cone1=self.calibrated_position, cone2=self.relative_position)

        if position_delta >= 0.5 and len(self.cones_clustered) > 0:
            self.last_x_offset, self.last_y_offset = self.calibrate_position(labels=labels, lidar_data=laser, position=rel_pos)
            self.calibrated_position = [self.relative_position[0] + self.last_x_offset,
                                        self.relative_position[1] + self.last_y_offset,
                                        self.relative_position[2]]
            self.get_logger().log(f"Offset between relative pos and calibrated pos is "
                                  f"{self.get_euclidean_distance(self.relative_position, self.calibrated_position)}")
            self.relative_position = self.calibrated_position
            
        """

        cones = self.received_labels(cone_labels, lidar_data, self.position)

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
                self.cones_clustered = self.use_dbscan(self.cones_clustered, _min_samples=1, _eps=0.1)

                cone_knowledge = [[], [], []]
                for cone_known in self.cones_clustered:
                    cone_knowledge[int(cone_known[3])].append(cone_known)
                self.track = self.calculate_track(cone_knowledge)
                self.autonomous_track = self.calculate_drive(self.track)
                print(self.start)
                print(self.start_arrived)
                if self.start_arrived: # wir waren bereits am Start, also an den orangen Cones
                    self.next_drive_commands = self.calculate_next_drive_commands(self.autonomous_track,
                                                                                  self.position)
                else:
                    self.next_drive_commands = self.start
                    self.start_arrived = True

                # publish the track_message
                track_message = Track()
                track_message.start = odom
                drive_x = [checkpoint[0] for checkpoint in self.next_drive_commands]
                drive_y = [checkpoint[1] for checkpoint in self.next_drive_commands]
                track_message.x = drive_x
                track_message.y = drive_y

                self.trackpublisher.publish(track_message)
                self.drive = True

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

    def calculate_drive(self, track):
        if len(track) == 2:
            blue_track = track[0].copy()
            yellow_track = track[1].copy()
            out = []
            for b_cone in blue_track:
                y_cone, y_dist = self.get_nearest_cone(b_cone, yellow_track)
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

        checkpoint_commands = []

        while len(auto_track) - 1 > next_checkpoint:
            checkpoint_commands.append(auto_track[next_checkpoint])
            next_checkpoint += 1

        return checkpoint_commands

    def get_nearest_cone(self, reference_cone: list, cone_list: List[list]):

        if len(cone_list) == 0:
            return None
        nearest_cone = cone_list[0]
        closest_distance = self.get_euclidean_distance(reference_cone, nearest_cone)
        # print("ref_cone: ", reference_cone, "nearest cone: ", nearest_cone, "clostest distnace: ", closest_distance)
        for listed_cone in cone_list:
            # print(closest_distance)
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

        position = [-odom.pose.pose.position.x, -odom.pose.pose.position.y, np.rad2deg(y)]

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
        self.ax.set_ylim(-2 + +Y[0], 2 + Y[0])
        self.ax.set_xlim(-2 + X[0], 2 + X[0])
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

        if len(self.track) == 2:
            blue_track = self.track[0]
            yellow_track = self.track[1]
            blue_x = [cone[0] for cone in blue_track]
            blue_y = [cone[1] for cone in blue_track]
            yellow_x = [cone[0] for cone in yellow_track]
            yellow_y = [cone[1] for cone in yellow_track]
            self.ax.plot(blue_x, blue_y, color='blue')
            self.ax.plot(yellow_x, yellow_y, color='yellow')

        track_x = []
        track_y = []
        for checkpoint in self.next_drive_commands:
            track_x.append(checkpoint[0])
            track_y.append(checkpoint[1])

        self.ax.scatter(track_x, track_y, color='tab:gray')
        self.ax.plot(track_x, track_y, color='tab:gray')

        # plt.show() # for testing at home

        plt.pause(.1) # normal code when working with the bot

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

    def calibrate_position(self, labels, lidar_data, position, block_size=2):
        received_cones_without_calibration = self.received_labels(labels, lidar_data, position)
        clustered_cones_without_calibration = self.use_dbscan(data_set=received_cones_without_calibration)

        # Dictionary by x. Cones are added to one key, if their x value rounded to the closest multiple of 2 is same
        # For example: (4.2, 3) and (5.3, 10) are added to one key 4 (as 4.2 and 5.3 rounded to the closest multiple
        # are 4)
        clustered_cones_wc_dict = OrderedDict()
        for cone in clustered_cones_without_calibration:
            x_key_rounded = (cone[0] // block_size) * block_size
            if x_key_rounded not in clustered_cones_wc_dict:
                clustered_cones_wc_dict[x_key_rounded] = []
            clustered_cones_wc_dict[x_key_rounded].append(cone)
        x_keys = list(clustered_cones_wc_dict.keys())

        x_offset, y_offset = [], []

        for clustered_cone in self.cones_clustered:
            # Finding the nearest cone by x
            ind = bisect.bisect_left(x_keys, clustered_cone[0])
            nearest_x = x_keys[max(ind - 1, 0):ind + 1]
            nearest_cones = []
            for x in nearest_x:
                nearest_cones += clustered_cones_wc_dict[x]
            nearest_cone, closest_distance = self.get_nearest_cone(clustered_cone, nearest_cones)
            x_offset.append(clustered_cone[0] - nearest_cone[0])
            y_offset.append(clustered_cone[1] - nearest_cone[1])
        x_offset_mean, y_offset_mean = np.mean(x_offset), np.mean(y_offset)

        return x_offset_mean, y_offset_mean

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
