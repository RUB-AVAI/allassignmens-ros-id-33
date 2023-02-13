import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations
import numpy as np


class PathIntegration(Node):

    def __init__(self):
        super().__init__('path_integration_node')

        self.delta_t = 0.05

        self.vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.timer = self.create_timer(self.delta_t, self.timer_callback)
        self.position_publisher = self.create_publisher(Odometry, '/codom', 10)

        self.vel = Twist()
        self.odom = Odometry()

    def timer_callback(self):
        # TODO: path integration
        # Distinct between cases: standing still, turning on spot or drivinng straight or driving curve

        _, _, yaw = tf_transformations.euler_from_quaternion(
            [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
             self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])

        position = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]

        delta_x = 0
        delta_y = 0
        delta_yaw = 0

        if self.vel.linear.x == 0 and self.vel.angular.z == 0:
            pass
        elif self.vel.angular.z == 0:
            # drive forward
            distance = self.vel.linear.x * self.delta_t
            delta_x = distance * np.cos(yaw)
            delta_y = distance * np.sin(yaw)
        elif self.vel.linear.x == 0:
            # turning on the spot
            delta_yaw = self.delta_t * self.vel.angular.z
        else:
            # driving curve
            radius = self.vel.linear.x / self.vel.angular.z
            delta_yaw = self.delta_t * self.vel.angular.z
            delta_x_ego = radius * np.sin(delta_yaw)
            delta_y_ego = radius * (1 - np.cos(delta_yaw))

            delta_x = delta_x_ego * np.cos(yaw) - delta_y_ego * np.sin(yaw)
            delta_y = delta_x_ego * np.sin(yaw) + delta_y_ego * np.cos(yaw)

        position = [position[0] + delta_x, position[1] + delta_y]
        yaw += delta_yaw
        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw)

        self.odom.pose.pose.position.x = position[0]
        self.odom.pose.pose.position.y = position[1]
        self.odom.pose.pose.orientation.x = quaternion[0]
        self.odom.pose.pose.orientation.y = quaternion[1]
        self.odom.pose.pose.orientation.z = quaternion[2]
        self.odom.pose.pose.orientation.w = quaternion[3]

        self.position_publisher.publish(self.odom)

    def vel_callback(self, data):
        self.vel = data


def main(args=None):
    rclpy.init(args=args)
    node = PathIntegration()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
