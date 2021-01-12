"""
Subscribes to both camera topic (/left_image and /right_image) and creates
a grpc server from where it can be accessed.
"""
from concurrent.futures import ThreadPoolExecutor
from threading import Lock
import rclpy
from rclpy.node import Node

import grpc
import cv2 as cv

from reachy_sdk_api import camera_pb2_grpc
from reachy_sdk_api.camera_pb2 import Image

from sensor_msgs.msg._compressed_image import CompressedImage
from cv_bridge import CvBridge


class CameraSubscriber(Node):
    """Subscriber to camera topics."""
    def __init__(self) -> None:
        super().__init__(node_name='camera_subscriber')
        self.clock = self.get_clock()
        self.left_cam_sub = self.create_subscription(
            CompressedImage,
            'left_image',
            self.decode_left_img,
            1)
        self.right_cam_sub = self.create_subscription(
            CompressedImage,
            'right_image',
            self.decode_right_img,
            1)
        self.bridge = CvBridge()

    def decode_left_img(self, msg):
        """Callback for /left_image subscriber."""
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, img = cv.imencode('.JPEG', img)
        self.left_image = img

    def decode_right_img(self, msg):
        """Callback for /right_image subscriber."""
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, img = cv.imencode('.JPEG', img)
        self.right_image = img


class CameraSDKServer(camera_pb2_grpc.CameraServiceServicer):
    """Camera server class."""
    def __init__(self) -> None:
        rclpy.init()
        self.cam_sub = CameraSubscriber()

        self.lock = Lock()
        self.both_images = {
             'left': self.cam_sub.left_image,
             'right': self.cam_sub.right_image
             }

    def GetImage(self, request, context):
        img_requested = self.both_images[request.side]
        with self.lock:
            rclpy.spin_once(self.cam_sub)
        img_msg = Image()
        img_msg.data = img_requested.image.tobytes()
        return img_msg


if __name__ == "__main__":
    opt = [('grpc.max_send_message_length', 200000), ('grpc.max_receive_message_length', 200000)]
    server = grpc.server(ThreadPoolExecutor(max_workers=10), options=opt)
    camera_pb2_grpc.add_CameraServiceServicer_to_server(CameraSDKServer(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    server.wait_for_termination()
