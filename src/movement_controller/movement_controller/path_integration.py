import builtin_interfaces
import rclpy
from rclpy.node import Node
import time
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
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.vel = Twist()
        self.odom = Odometry()

        self.odom_start = None

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
        t = builtin_interfaces.msg.Time()
        now = time.time()
        sec = int(now)
        nsec = int((now - sec) * 10 ** 9)
        t.sec = sec
        t.nanosec = nsec
        self.odom.header.stamp = t
        out = "path_integration: x: " + str(self.odom.pose.pose.position.x) + " y: " + str(self.odom.pose.pose.position.y) + " rotation: " + str(np.rad2deg(yaw))
        self.get_logger().info(out)
        self.position_publisher.publish(self.odom)

    def vel_callback(self, data):
        self.vel = data

    def odom_callback(self, data):
        if self.odom_start == None:
            self.odom_start = data
        delta_x = self.odom_start.pose.pose.position.x - data.pose.pose.position.x
        delta_y = self.odom_start.pose.pose.position.y - data.pose.pose.position.y
        out = "odom_topic: x: " + str(delta_x) + " y: " + str(delta_y)
        #self.get_logger().info(out)


def main(args=None):
    rclpy.init(args=args)
    node = PathIntegration()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
