import time

import builtin_interfaces
import rclpy
import cv2
from rclpy.node import Node
from rclpy.timer import Timer
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import os


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.timer_period = 0.  # seconds
        self.cap = cv2.VideoCapture(0)
        # self.cap = cv2.imread("/home/ubuntu/validation/images/12.png")
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/images/raw', 10)
        self.subscription_ = self.create_subscription(Float64, '/camera/freq', self.camera_ctrl_callback, 10)
        self.shutter_subscription = self.create_subscription(Bool, '/camera/shutter', self.camera_shutter_callback, 10)
        self.timer = self.create_timer(100, self.capture_image_callback)
        self.timer.cancel() # to start with 0Hz

    def capture_image_callback(self):
        ret, frame = self.cap.read()
        #ret = True
        #frame = self.cap
        if ret:
            imgmsg = self.bridge.cv2_to_imgmsg(frame)
            t = builtin_interfaces.msg.Time()
            now = time.time()
            sec = int(now)
            nsec = int((now - sec)*10**9)
            t.sec = sec
            t.nanosec = nsec
            imgmsg.header.stamp = t
            self.publisher_.publish(imgmsg)
            self.get_logger().info(f"raw image ->")

    def camera_ctrl_callback(self, msg):
        if msg.data == 0.:
            self.timer.cancel()
            self.get_logger().warn("<- stoppping timer")
        else:
            self.timer_period = msg.data
            self.timer.cancel()
            self.timer = self.create_timer(self.timer_period, self.capture_image_callback)
            self.get_logger().info(f"<- set camera timer period to {self.timer_period}")

    def camera_shutter_callback(self, msg):
        if msg.data:
            self.capture_image_callback()
        else:
            self.get_logger().warn("wrong shutter command")


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)

    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
