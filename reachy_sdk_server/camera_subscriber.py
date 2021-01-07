import cv2 as cv

from rclpy.node import Node

from sensor_msgs.msg._compressed_image import CompressedImage
from cv_bridge import CvBridge


class CameraSubcriber(Node):
    def __init__(self, node_name: str, topic: str) -> None:
        super().__init__(node_name=node_name)
        self.clock = self.get_clock()
        self.camera_sdk_subscriber = self.create_subscription(
            CompressedImage,
            'left_image',
            self.decode_img,
            1)
        self.bridge = CvBridge()

    def decode_img(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, img = cv.imencode('.JPEG', img)
        self.image = img