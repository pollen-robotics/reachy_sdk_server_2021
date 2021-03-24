"""Expose Reachy ROS services/topics dealing with camera and zoom controlling through gRPC allowing remote client SDK."""

import time
from functools import partial

from concurrent.futures import ThreadPoolExecutor
from threading import Event
from typing import Iterator

import grpc

import rclpy
from rclpy.node import Node

from sensor_msgs.msg._compressed_image import CompressedImage
from reachy_msgs.srv import GetCameraZoomLevel, GetCameraZoomSpeed
from reachy_msgs.srv import SetCameraZoomLevel, SetCameraZoomSpeed

from reachy_sdk_api import camera_reachy_pb2, camera_reachy_pb2_grpc


class CameraServer(
                Node,
                camera_reachy_pb2_grpc.CameraServiceServicer,
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

        self.get_zoom_level_client = self.create_client(GetCameraZoomLevel, 'get_camera_zoom_level')
        self.get_zoom_speed_client = self.create_client(GetCameraZoomSpeed, 'get_camera_zoom_speed')
        self.set_zoom_level_client = self.create_client(SetCameraZoomLevel, 'set_camera_zoom_level')
        self.set_zoom_speed_client = self.create_client(SetCameraZoomSpeed, 'set_camera_zoom_speed')

        self.left_camera_sub = self.create_subscription(
            CompressedImage,
            'left_image',
            partial(self.on_image_update, side='left'),
            1,
        )

        self.right_camera_sub = self.create_subscription(
            CompressedImage,
            'right_image',
            partial(self.on_image_update, side='right'),
            1,
        )

        self.cam_img = {
            'left': None,
            'right': None
        }
        self.image_published = {
            'left': Event(),
            'right': Event(),
        }

        self.logger.info('Camera server ready!')

    def on_image_update(self, msg, side):
        """Get data from image. Callback for "/'side'_image "subscriber."""
        self.cam_img[side] = msg.data.tobytes()
        self.image_published[side].set()

    def _wait_for(self, future):
        for _ in range(10000):
            if future.done():
                return future.result()
            time.sleep(0.001)

    # Handle GRPCs
    # Camera Image
    def GetImage(self, request: camera_reachy_pb2.ImageRequest, context) -> camera_reachy_pb2.Image:
        """Get the image from the requested camera topic."""
        side = 'left' if camera_reachy_pb2.ImageRequest.camera == camera_reachy_pb2.CameraId.LEFT else 'right'

        im_msg = camera_reachy_pb2.Image()
        im_msg.data = self.cam_img[side]

        return im_msg

    def StreamImage(self, request: camera_reachy_pb2.StreamImageRequest, context) -> Iterator[camera_reachy_pb2.Image]:
        """Stream the image from the requested camera topic."""
        side = 'left' if camera_reachy_pb2.ImageRequest.camera == camera_reachy_pb2.CameraId.LEFT else 'right'

        while True:
            self.image_published[side].wait()
            yield self.GetImage(request.request, context)
            self.image_published[side].clear()

    def GetZoomLevel(self, request: camera_reachy_pb2.Camera, context) -> camera_reachy_pb2.ZoomLevel:
        """Return current zoom level."""
        req = GetCameraZoomLevel.Request()
        req.name = 'left_eye' if request.id == camera_reachy_pb2.CameraId.LEFT else 'right_eye'
        result = self._wait_for(self.get_zoom_level_client.call_async(req))

        return camera_reachy_pb2.ZoomLevel(
            level=getattr(camera_reachy_pb2.ZoomLevelPossibilities, result.zoom_level.upper())
        )

    def GetZoomSpeed(self, request: camera_reachy_pb2.Camera, context) -> camera_reachy_pb2.ZoomSpeed:
        """Return current zoom speed."""
        req = GetCameraZoomSpeed.Request()
        req.name = 'left_eye' if request.id == camera_reachy_pb2.CameraId.LEFT else 'right_eye'
        result = self._wait_for(self.get_zoom_speed_client.call_async(req))

        return camera_reachy_pb2.ZoomSpeed(
            speed=result.speed,
        )

    def SendZoomCommand(self, request: camera_reachy_pb2.ZoomCommand, context) -> camera_reachy_pb2.ZoomCommandAck:
        """Handle zoom command."""
        name = 'left_eye' if request.camera.id == camera_reachy_pb2.CameraId.LEFT else 'right_eye'

        if request.HasField('homing_command'):
            req = SetCameraZoomLevel.Request()
            req.name = name
            req.zoom_level = 'homing'
            result = self._wait_for(self.set_zoom_level_client.call_async(req))
            success = True if result is not None else False
            return camera_reachy_pb2.ZoomCommandAck(success=success)

        elif request.HasField('level_command'):
            req = SetCameraZoomLevel.Request()
            req.name = name
            req.zoom_level = camera_reachy_pb2.ZoomLevelPossibilities.Name(request.level_command.level).lower()
            result = self._wait_for(self.set_zoom_level_client.call_async(req))
            success = True if result is not None else False
            return camera_reachy_pb2.ZoomCommandAck(success=success)

        elif request.HasField('speed_command'):
            req = SetCameraZoomSpeed.Request()
            req.name = name
            req.speed = request.speed_command.speed
            result = self._wait_for(self.set_zoom_speed_client.call_async(req))
            success = True if result is not None else False
            return camera_reachy_pb2.ZoomCommandAck(success=success)

        return camera_reachy_pb2.ZoomCommandAck(success=False)


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
