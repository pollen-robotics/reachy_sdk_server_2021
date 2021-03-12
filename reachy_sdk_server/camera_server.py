"""Expose Reachy ROS services/topics dealing with camera and zoom controlling through gRPC allowing remote client SDK."""

from functools import partial

from concurrent.futures import ThreadPoolExecutor

import grpc

import rclpy
from rclpy.node import Node

from sensor_msgs.msg._compressed_image import CompressedImage
from reachy_msgs.srv import SetCameraZoomLevel, SetCameraZoomSpeed

from reachy_sdk_api import camera_reachy_pb2 as cam_pb, camera_reachy_pb2_grpc
from reachy_sdk_api import zoom_command_pb2 as zoom_pb, zoom_command_pb2_grpc


class CameraServer(
                Node,
                camera_reachy_pb2_grpc.CameraServiceServicer,
                zoom_command_pb2_grpc.ZoomControllerServiceServicer,
                ):
    """Camera server node."""

    def __init__(self, node_name: str) -> None:
        """Set up the node.

        Subscribe to image topics (/left_image and /right_image).
        Setup zoom services.
        """
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()

        self.clock = self.get_clock()

        self.logger.info('Launching sub/srv...')

        self.zoom_level_client = self.create_client(SetCameraZoomLevel, 'set_camera_zoom_level')
        self.zoom_speed_client = self.create_client(SetCameraZoomSpeed, 'set_camera_zoom_speed')

        self.left_camera_sub = self.create_subscription(
            CompressedImage,
            'left_image',
            partial(self.decode_img, side='left'),
            1)
        self.right_camera_sub = self.create_subscription(
            CompressedImage,
            'right_image',
            partial(self.decode_img, side='right'),
            1)
        self.cam_img = {
            'left': None,
            'right': None
        }

        self.logger.info('Camera server ready!')

    def decode_img(self, msg, side):
        """Get data from image. Callback for "/'side'_image "subscriber."""
        self.cam_img[side] = msg.data

    # Handle GRPCs
    # Camera Image
    def GetImage(self, request, context):
        """Get the image from the requested camera topic."""
        im_msg = cam_pb.Image()
        im_msg.data = self.cam_img[request.side].tobytes()
        return im_msg

    # Zoom Controller
    def SendZoomCommand(self, request, context):
        """Send command to zoom controller of the requested camera."""
        req = SetCameraZoomLevel.Request()
        req.name = request.side
        req.zoom_level = request.command
        _ = self.zoom_level_client.call_async(req)
        return zoom_pb.Empty()

    def SetZoomSpeed(self, request, context):
        """Change zoom controller motors speed."""
        req = SetCameraZoomSpeed.Request()
        req.speed = request.speed
        _ = self.zoom_speed_client.call_async(req)
        return zoom_pb.Empty()


def main():
    """Run the Node and the gRPC server."""
    rclpy.init()

    camera_server = CameraServer(node_name='camera_server')

    options = [
         ('grpc.max_send_message_length', 250000),
         ('grpc.max_receive_message_length', 250000),
         ]

    server = grpc.server(thread_pool=ThreadPoolExecutor(max_workers=10), options=options)
    camera_reachy_pb2_grpc.add_CameraServiceServicer_to_server(camera_server, server)
    zoom_command_pb2_grpc.add_ZoomControllerServiceServicer_to_server(camera_server, server)

    server.add_insecure_port('[::]:50057')
    server.start()

    try:
        rclpy.spin(camera_server)
    except KeyboardInterrupt:
        pass

    server.stop(grace=None)
    server.wait_for_termination()

    camera_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
