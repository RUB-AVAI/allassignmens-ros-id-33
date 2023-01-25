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
from .icp import icp


class ConeLocalizationNode(Node):

    def __init__(self):
        super().__init__('cone_localization_node')

        self.cones_new = []
        self.cones_clustered = []
        self.lidar_data = None # should be numpy.ndarray with shape (360, 2)
        self.odom_data = None # Odometry.pose.pose
        self.position = [0, 0] # robots position in meters
        self.orientation = 0 # robots head orientation in radians
        # self.sign = -1

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

        # TODO: initialize position and orientation if still None

        cone_labels = labels.cones

        cones = self.received_labels(cone_labels, lidar_data, [self.position[0], self.position[1], np.rad2deg(self.orientation)]) # TODO: also calculate position here like done in synchronized callback
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
            self.count_for_DBSCAN = 0


    def use_dbscan(self, data_set, _min_samples=2, _eps=.1):
        # clustering over x, y, color
        x_train = []
        # We set the z coordinate of the cones points in DBSCAN as DIGIT_FOR_COLOR * (_eps + 1)
        # to prevent clustering cones of different color to one.
        # See report of assignment 7 for detailed explanation.
        for c in data_set:
            x_train.append([c[0], c[1], c[3] * (_eps + 1)])

        dbscan = DBSCAN(eps=_eps, min_samples=_min_samples).fit(x_train)

        # in cluster_lables sind die entsprechenden Labels für jeden Eintrag --> neuer Wert --> neues Hütchen
        cluster_labels = dbscan.labels_

        DBSCAN_dataset = data_set.copy()
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
        """
        Callback on laser and odometry data. Tries to keep track of position with ICP.
        :param laser:
        :param odom:
        :return:
        """
        lidar_data = np.asarray(laser.ranges[::-1])
        odom_data = odom.pose.pose

        # initialize odom_data if it is first call of function
        if self.odom_data is None:
            self.odom_data = odom_data

        # calculate estimated new pos
        delta_x = odom_data.position.x - self.odom_data.position.x
        delta_y = odom_data.position.y - self.odom_data.position.y

        _, _, yaw_old = tf_transformations.euler_from_quaternion([self.odom_data.orientation.x, self.odom_data.orientation.y, self.odom_data.orientation.z,
             self.odom_data.orientation.w])
        _, _, yaw_new = tf_transformations.euler_from_quaternion([odom_data.orientation.x, odom_data.orientation.y, odom_data.orientation.z,
             odom_data.orientation.w])
        delta_yaw = yaw_old - yaw_new # yaw in radians

        estimated_pos = [self.position[0] + delta_x, self.position[1] + delta_y]
        estimated_yaw = self.orientation + delta_yaw

        l_data = []
        for angle in range(len(lidar_data)):
            distance = lidar_data[angle]
            x, y = self.get_euclidean_coordinates(angle, distance)
            l_data.append([x, y])

        # needs to be np.array(360, 2) in order for ICP to work (reshape might not be needed)
        l_data = np.asarray(l_data) #.reshape(360, 2)

        # initialize lidar_data on first function call
        if self.lidar_data is None:
            print(self.lidar_data)
            self.lidar_data = l_data

        # do icp and compare translation and rotation
        transformation_history, _ = icp(reference_points=l_data,
                                        points=self.lidar_data,
                                        max_iterations=100,
                                        distance_threshold=1e-3,
                                        convergence_translation_threshold=1,
                                        convergence_rotation_threshold=1,
                                        point_pairs_threshold=150,
                                        verbose=True)

        # get one transformation matrix out of transformation_history (translation + rotation)
        # Note: rotation and translation are commutativ
        transformation_matrix = np.eye(3, 3)
        for transform in transformation_history:
            trans_mat = np.eye(3, 3)
            trans_mat[:2, :3] = transform
            transformation_matrix = transformation_matrix @ trans_mat

        print(f"position delta odom; {delta_x}, {delta_y}, {delta_yaw}")
        delta_x = transformation_matrix[2, 0]
        delta_y = transformation_matrix[2, 1]
        delta_yaw = np.arcsin(transformation_matrix[0, 1])
        print(f"position delta odom; {delta_x}, {delta_y}, {delta_yaw}")
        # TODO: compare them and get some error value

        # TODO: what to do if error too big? Only use odom??

        self.position = [self.position[0] + delta_x, self.position[1] + delta_y]
        self.orientation = self.orientation + delta_yaw


        self.lidar_data = l_data
        self.draw()

    def draw(self):
        """ Plots the current situation with matplotlib """
        # clear the axis (the plot)
        self.ax.cla()

        # draw robot
        self.ax.scatter(self.position[0], self.position[1], color='black', alpha=1)

        # draw the lidar data
        lidar_x = []
        lidar_y = []
        for entry in self.lidar_data:
            # rotate and translate according to robots position and orientation
            c, s = np.cos(self.orientation), np.sin(self.orientation)
            tmp_x = c * entry[0] - s * entry[1]
            tmp_y = s * entry[0] + c * entry[1]
            tmp_x += self.position[0]
            tmp_y += self.position[1]
            lidar_x.append(tmp_x)
            lidar_y.append(tmp_y)

        self.ax.scatter(lidar_x, lidar_y, s=.4)
        self.ax.set_ylim(-2, 2)
        self.ax.set_xlim(-2, 2)
        line_x = []
        line_y = []
        dist = np.linspace(0, 2, 100)

        # draw robots fov to show direction
        orientation = np.rad2deg(self.orientation) # get orientation in radians

        for d in dist:
            lx, ly = self.get_euclidean_coordinates((148 - orientation) % 360, d)
            line_x.append(lx + self.position[0])
            line_y.append(ly + self.position[1])

        for d in dist:
            lx, ly = self.get_euclidean_coordinates((212 - orientation) % 360, d)
            line_x.append(lx + self.position[0])
            line_y.append(ly + self.position[1])

        self.ax.plot(line_x, line_y, color='green')

        # draw the clustered cones (the robots knowledge)
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

        # draw the cones that are not yet clustered
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
