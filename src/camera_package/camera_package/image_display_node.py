import sys
import os
import cv2
from cv_bridge import CvBridge
from PyQt5 import QtWidgets, QtGui
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool
from threading import Thread
from .hmi import Ui_MainWindow
from datetime import datetime, timezone


class ImageDisplayNode(Node):
    # simple node to show images from image_processing_node and to control camera_node

    def __init__(self, ui):
        super().__init__('image_display_node')

        self.ui = ui
        self.ui.shutterBtn.clicked.connect(self.camera_shutter_callback_)
        self.ui.updateCameraFreqBtn.clicked.connect(self.camera_freq_publisher_callback_)

        self.processed_images_subscription_ = self.create_subscription(
            Image, '/images/processed', self.processed_callback, 10)

        # uncomment to see raw images as well note that this will use more bandwidth
        self.raw_images_subscription_ = self.create_subscription(Image, 'raw_images', self.raw_callback, 10)

        self.camera_ctrl_publisher_ = self.create_publisher(Float64, '/camera/freq', 10)
        self.shutter_publisher_ = self.create_publisher(Bool, '/camera/shutter', 10)
        self.bridge = CvBridge()

    def camera_freq_publisher_callback_(self):
        # set the camera freq
        if not self.ui.lineEdit.text():
            self.get_logger().warn("no input given")
        else:
            camera_freq = float(self.ui.lineEdit.text())
            msg = Float64()
            if camera_freq == 0.:
                msg.data = camera_freq
            else:
                msg.data = 1. / camera_freq
            self.camera_ctrl_publisher_.publish(msg)
            self.get_logger().info(f'camera timer period: {msg.data} ->')

    def camera_shutter_callback_(self):
        # take a single foto
        msg = Bool()
        msg.data = True
        self.shutter_publisher_.publish(msg)
        self.get_logger().info(f'camera shutter command ->')

    def processed_callback(self, data):
        # show images in ui and persist to hardrive
        self.get_logger().info('<- processed image')
        frame = self.bridge.imgmsg_to_cv2(data)
        qt_image = QtGui.QImage(frame.data, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
        self.ui.image_frame.setPixmap(QtGui.QPixmap.fromImage(qt_image))
        filename = "img/" + str(int(datetime.now(timezone.utc).timestamp() * 1000000)) + ".png"
      #  cv2.imwrite(filename, frame)

    def raw_callback(self, data):
        self.get_logger().info('<- raw image')
        frame = self.bridge.imgmsg_to_cv2(data)
        qt_image = QtGui.QImage(frame.data, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
        self.ui.raw_frame.setPixmap(QtGui.QPixmap.fromImage(qt_image))
        # filename = "img/raw/" + str(int(datetime.now(timezone.utc).timestamp() * 1000000)) + ".png"
        # cv2.imwrite(filename, frame)


def main(args=None):
    # check if the img directory is created
    path = 'img'
    if not os.path.exists(path):
        os.mkdir(path)

    rclpy.init(args=args)

    # setup ui
    app = QtWidgets.QApplication(sys.argv)
    hmi = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(hmi)

    # handle node in thread parallel to ui
    image_display_node = ImageDisplayNode(ui)
    executor = SingleThreadedExecutor()
    executor.add_node(image_display_node)

    thread = Thread(target=executor.spin)

    try:
        thread.start()
        hmi.show()
        sys.exit(app.exec_())

    finally:
        image_display_node.destroy_node()
        executor.shutdown()
        thread.join()


if __name__ == '__main__':
    main()
