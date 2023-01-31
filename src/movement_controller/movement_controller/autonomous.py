import rclpy
from rclpy.node import Node
import numpy as np
import math

from avai_messages.msg import Track
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class AutonomousController(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.track_subscriber = self.create_subscription(Track, '/track', self.received_track_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.received_odom_callback, 10)

        self.start_position = None
        self.next_target = None
        self.target_queue = None
        self.threshold = 0.05 # 5cm

        
    def received_track_callback(self, track):
        self.get_logger().info("<- new track")
        self.start_position = track.start
        goal_x = track.x
        goal_y = track.y
        self.target_queue = [[x, goal_y[idx]] for idx, x in enumerate(goal_x)]
        self.next_target = self.target_queue.pop()

    def received_odom_callback(self, odom):
        if not self.next_target is None:
            # if position is given relative have to check that
            delta_x = odom.pose.pose.position.x - self.start_position.pose.pose.position.x
            delta_y = odom.pose.pose.position.y - self.start_position.pose.pose.position.y
            target_distance = np.sqrt((self.next_target[0] - delta_x) ** 2 + (self.next_target[1] - delta_y) ** 2)

            if target_distance <= self.threshold:
                self.get_logger().info("reached next target point")
                if len(self.target_queue) >= 1:
                    self.next_target = self.target_queue.pop()
                else:
                    self.next_target = None
                    self.speed_publisher.publish(Twist()) # values should all be zero
                    self.get_logger().info("completed track")
                    return
            
            _, _, new_yaw = tf_transformations.euler_from_quaternion(
            [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
             odom.pose.pose.orientation.w])
            _, _, old_yaw = tf_transformations.euler_from_quaternion(
            [self.start_position.pose.pose.orientation.x, self.start_position.pose.pose.orientation.y, self.start_position.pose.pose.orientation.z,
             oself.start_positiondom.pose.pose.orientation.w])
            delta_yaw = new_yaw - old_yaw

            angle_to_target = self.angle_from_points([delta_x, delta_y], self.next_target)

            angular_vel = self.target_dynamic(delta_yaw, angle_to_target, 1.) 
            twist = Twist()
            twist.linear.x = 1
            twist.angular.z = angular_vel
            self.speed_publisher.publish(twist)


    @staticmethod
    def target_dynamic(robot_orientation, target_orientation, turning_speed):
        # to avoid being stuck at 180deg relative to the target. Because rate of change directly on a repeller is 0.
        if abs(AutonomousController.normalize_radians(robot_orientation - target_orientation) - np.pi) < 0.01:
            return np.pi * .25
        return -turning_speed * np.sin(robot_orientation - target_orientation)

    @staticmethod
    def normalize_radians(radians):
        rad = radians
        if rad > np.pi:
            rad -= 2. * np.pi
        elif rad < -np.pi:
            rad += 2. * np.pi
        return rad

    @staticmethod(f)
    def angle_from_points(a, b):
        z = a[0] - b[0]
        x = a[1] - b[1]
        return math.atan2(x, z)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
