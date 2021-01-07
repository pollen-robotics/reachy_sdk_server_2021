"""
Subscribes to a camera topic (/left_image or /right_image) and creates 
a grpc server from where it can be accessed.
"""

import grpc
import cv2 as cv


import rclpy
from rclpy.node import Node

from concurrent.futures import ThreadPoolExecutor
from reachy_sdk_api import camera_pb2 as cam_pb, camera_pb2_grpc

from sensor_msgs.msg._compressed_image import CompressedImage
from cv_bridge import CvBridge


class Camera(Node):
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


class CameraSDKServer(camera_pb2_grpc.CameraServiceServicer):
    def __init__(self) -> None:
        self.cam = Camera(node_name='camera_subscriber', topic='left_image')

    def GetImage(self, request, context):
        rclpy.spin_once(self.cam)
        imMsg = cam_pb.Image()
        imMsg.image = self.cam.image.tobytes()
        return imMsg


if __name__ == "__main__":
    rclpy.init()

    options = [
        ('grpc.max_send_message_length', 512 * 1024 * 1024),
        ('grpc.max_receive_message_length', 512 * 1024 * 1024)]
    server = grpc.server(ThreadPoolExecutor(max_workers=10), options=options)
    camera_pb2_grpc.add_CameraServiceServicer_to_server(CameraSDKServer(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    server.wait_for_termination()
