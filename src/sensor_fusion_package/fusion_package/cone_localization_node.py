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
        #print("x: ", position[0], "y: ", position[1])

        if len(self.startup_position) == 0:
            self.startup_position = position

        rel_pos = []
        rel_pos.append(position[0] - self.startup_position[0])
        rel_pos.append(position[1] - self.startup_position[1])
        rel_pos.append((self.startup_position[2] - position[2]) % 360)
        # self.relative_position[0] = self.startup_position[0] - position[0]
        # self.relative_position[1] = self.startup_position[1] - position[1]
        # self.relative_position[2] = (self.startup_position[2] + position[2]) % 360
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

    def use_dbscan(self, data_set, _min_samples=2):

        # clustering over x, y, color
        # TODO: Check whether values are saved correctly for clustering!
        x_train = []
        for c in data_set:
            x_train.append([c[0], c[1], c[3]])

        # variables
        _eps = .5  # max distance to be considered in neighborhood
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
        """
        # every label only once in the set(cluster_labels) to get the amount of different cones we clustered
        unique_cone_labels = set(cluster_labels)
        # creating dictionary with keys as labels and values as lists of cones
        clustered_cones = dict([(label, []) for label in unique_cone_labels])
        for elem in DBSCAN_dataset:
            # elem[-1] is the label (see concatenation of DBSCAN_dataset)
            # elem[0:3] are x, y and cone color respectively
            clustered_cones[elem[-1]].append(elem[0:3])
                """

        # clustered_cones enthält jetzt für jedes clustered cone eine Liste an Positionen. clustered[0] =  "outliers"
        new_cone_representation = []
        for cluster in clustered_cones:
            # avoiding cones with label == -1 as noise
            # question: do we have to check len(cone) as len(cone) == 0 is actually impossible (if cone doesn't have
            # label then it is noise and has label -1 => all len(cones) are > 0).
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
        # self.relative_position[0] = self.startup_position[0] - position[0]
        # self.relative_position[1] = self.startup_position[1] - position[1]
        # self.relative_position[2] = (self.startup_position[2] + position[2]) % 360
        # self.relative_position[2] = (odom.pose.pose.orientation.z + 1) * 180

        l_data = []
        for angle in range(len(lidar_data)):
            distance = lidar_data[angle]

            x, y = self.get_euclidean_coordinates((angle - rel_pos[2]) % 360, distance)

            x += self.relative_position[0]
            y += self.relative_position[1]
            l_data.append([x, y])

        # print("l_data: ", l_data)
        """
        for idx, point in enumerate(lidar_data):
            lidar_data[idx] = point + self.position
        self.lidar_data = lidar_data
        """
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
        #print("x_pos:", self.relative_position[0], "y_pos: ", self.relative_position[1])
        self.ax.scatter(self.relative_position[0], self.relative_position[1], color='black', alpha=1)

        lidar_x = []
        lidar_y = []

        for entry in self.lidar_data:
            lidar_x.append(entry[0])
            lidar_y.append(entry[1])

        self.ax.scatter(lidar_x, lidar_y, s=.4)
        #self.ax.set_ylim(np.min(lidar_y), np.max(lidar_y))
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
            # print("distance: ", cone[1], "labelclass: ", cone[3], "angle: ", cone[0])
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
            # print("distance: ", cone[1], "labelclass: ", cone[3], "angle: ", cone[0])
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
        #print(len(self.cones_clustered))
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
                rotation_angle = np.deg2rad(delta_rotation)
                #print(position)
                #x = np.cos(rotation_angle)*x - np.sin(rotation_angle)*y
                #y = np.sin(rotation_angle)*x + np.cos(rotation_angle)*y
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
