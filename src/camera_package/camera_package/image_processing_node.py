import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageProcessingNode(Node):
    # simple node to process the images

    def __init__(self):
        super().__init__('image_processing_node')
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/images/processed', 10)
        self.subscription_ = self.create_subscription(Image, '/images/raw', self.process_image_callback, 10)

    def process_image_callback(self, data):
        self.get_logger().info('<- raw image')
        frame = self.bridge.imgmsg_to_cv2(data)

        # downsize the image and convert to grayscale, to save bandwidth
        scale = 2
        new_size = (frame.shape[1] // scale, frame.shape[0] // scale)
        frame = cv2.resize(frame, dsize=new_size, fx=1./scale, fy=1./scale)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame))
        self.get_logger().info('processed image ->')


def main(args=None):
    rclpy.init(args=args)
    image_processing_node = ImageProcessingNode()
    rclpy.spin(image_processing_node)
    image_processing_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()