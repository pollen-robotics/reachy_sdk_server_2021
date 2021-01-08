import cv2 as cv

from rclpy.node import Node

from sensor_msgs.msg._compressed_image import CompressedImage
from cv_bridge import CvBridge


class CameraSubscriber(Node):
    def __init__(self, side: str) -> None:
        super().__init__(node_name=side+'_cam_subscriber')
        self.clock = self.get_clock()
        self.cam_sub = self.create_subscription(
            CompressedImage,
            side+'_image',
            self.decode_img,
            1)
        self.bridge = CvBridge()

    def decode_img(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, img = cv.imencode('.JPEG', img)
        self.image = img