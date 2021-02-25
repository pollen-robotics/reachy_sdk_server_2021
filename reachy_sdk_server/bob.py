"""Expose main Reachy ROS services/topics through gRPC allowing remote client SDK."""

import time
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, Iterator, List
from threading import Event
from functools import partial

import numpy as np

from scipy.spatial.transform import Rotation

from google.protobuf.empty_pb2 import Empty
from google.protobuf.timestamp_pb2 import Timestamp

import grpc

import rclpy
from rclpy.node import Node

from reachy_msgs.msg import JointTemperature, LoadSensor
from reachy_msgs.srv import GetJointsFullState, SetCompliant
from reachy_msgs.srv import GetArmIK, GetArmFK, GetOrbitaIK, GetQuaternionTransform as GetQuatTf
from reachy_msgs.srv import ZoomCommand, SetZoomSpeed

from reachy_sdk_api import joint_command_pb2 as jc_pb, joint_command_pb2_grpc
from reachy_sdk_api import joint_state_pb2 as js_pb, joint_state_pb2_grpc
from reachy_sdk_api import camera_reachy_pb2 as cam_pb, camera_reachy_pb2_grpc
from reachy_sdk_api import load_sensor_pb2 as ls_pb, load_sensor_pb2_grpc
from reachy_sdk_api import orbita_kinematics_pb2 as orbita_pb, orbita_kinematics_pb2_grpc
from reachy_sdk_api import kinematics_pb2 as kin_pb
from reachy_sdk_api import arm_kinematics_pb2 as armk_pb, arm_kinematics_pb2_grpc
from reachy_sdk_api import cartesian_command_pb2 as cart_pb, cartesian_command_pb2_grpc
from reachy_sdk_api import zoom_command_pb2 as zc_pb, zoom_command_pb2_grpc

from sensor_msgs import msg
from sensor_msgs.msg._compressed_image import CompressedImage
from geometry_msgs.msg import Point, Quaternion

from .utils import jointstate_pb_from_request


protoside_to_str = {
    armk_pb.ArmSide.LEFT: 'left',
    armk_pb.ArmSide.RIGHT: 'right',
}


class Bob(Node):
    def __init__(self):
        super().__init__(node_name='jean_bob')

        # Kinematics
        self.left_arm_ik = self.create_client(GetArmIK, '/reachy_left_arm_kinematics_service/inverse')
        self.right_arm_ik = self.create_client(GetArmIK, '/reachy_right_arm_kinematics_service/inverse')

        from threading import Thread
        t = Thread(target=self.spam)
        t.start()


    def spam(self):
        import time
        import numpy as np

        while True:
            self._call_arm_ik()
            time.sleep(np.random.rand())
            #time.sleep(0.001)

    def _call_arm_ik(self):
        ik_client = self.left_arm_ik

        ros_req = GetArmIK.Request()
        M = np.random.rand(4, 4)#np.array(request.target.data).reshape((4, 4))

        ros_req.pose.position = Point(x=M[0, 3], y=M[1, 3], z=M[2, 3])
        q = Rotation.from_matrix(M[:3, :3]).as_quat()
        ros_req.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # if request.q0:
        # ros_req.q0.position = request.q0.positions

        return ik_client.call(ros_req)


def main():
    """Run the Node and the gRPC server."""
    rclpy.init()

    bob = Bob()

    try:
        rclpy.spin(bob)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
