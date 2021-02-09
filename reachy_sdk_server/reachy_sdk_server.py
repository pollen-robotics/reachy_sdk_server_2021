"""Expose main Reachy ROS services/topics through gRPC allowing remote client SDK."""

import time
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, Iterator
from threading import Event

from functools import partial

import cv2 as cv
import reachy_sdk_api

from cv_bridge import CvBridge

from google.protobuf.empty_pb2 import Empty
from google.protobuf.timestamp_pb2 import Timestamp

import grpc

import rclpy
from rclpy.node import Node

from reachy_msgs.msg import JointTemperature, LoadSensor
from reachy_msgs.srv import GetJointsFullState, SetCompliant, GetOrbitaIK

from reachy_sdk_api import joint_command_pb2 as jc_pb, joint_command_pb2_grpc
from reachy_sdk_api import joint_state_pb2 as js_pb, joint_state_pb2_grpc
from reachy_sdk_api import camera_reachy_pb2 as cam_pb, camera_reachy_pb2_grpc
from reachy_sdk_api import load_sensor_pb2 as ls_pb, load_sensor_pb2_grpc
from reachy_sdk_api import orbita_kinematics_pb2 as orbk_pb, orbita_kinematics_pb2_grpc
from reachy_sdk_api import kinematics_pb2 as kin_pb

from sensor_msgs import msg
from sensor_msgs.msg._compressed_image import CompressedImage

from .utils import jointstate_pb_from_request
from orbita_kinematics.orbita_kinematics import OrbitaKinSolver
from geometry_msgs.msg import Quaternion


class ReachySDKServer(Node,
                      joint_state_pb2_grpc.JointStateServiceServicer,
                      joint_command_pb2_grpc.JointCommandServiceServicer,
                      camera_reachy_pb2_grpc.CameraServiceServicer,
                      load_sensor_pb2_grpc.LoadServiceServicer,
                      orbita_kinematics_pb2_grpc.OrbitaKinematicServicer):
    """Reachy SDK server node."""

    def __init__(self, node_name: str, timeout_sec: float = 5, pub_frequency: float = 100) -> None:
        """Set up the node.

        Subscribe to /joint_state, /joint_temp, /force_gripper.
        Publish new command on /joint_goal or concerned services.

        """
        super().__init__(node_name=node_name)

        self.timeout_sec = timeout_sec
        self.pub_period = 1 / pub_frequency

        self.clock = self.get_clock()
        self.logger = self.get_logger()

        self.joints: Dict[str, Dict[str, float]] = OrderedDict()
        self.load_sensors: Dict[str, float] = OrderedDict()
        self.setup()
        self.id2names = {i: name for i, name in enumerate(self.joints.keys())}

        self.logger.info('Launching pub/sub/srv...')
        self.compliant_client = self.create_client(SetCompliant, 'set_compliant')
        self.orbita_ik_client = self.create_client(GetOrbitaIK, 'orbita_ik')

        self.joint_states_sub = self.create_subscription(
            msg_type=msg.JointState, topic='joint_states',
            callback=self.on_joint_states, qos_profile=5,
        )
        self.joint_state_pub_event = Event()
        self.joint_temperatures_sub = self.create_subscription(
            msg_type=JointTemperature, topic='joint_temperatures',
            callback=self.on_joint_temperatures, qos_profile=5,
        )
        self.load_sensor_sub = self.create_subscription(
            msg_type=LoadSensor, topic='force_gripper',
            callback=self.on_load_sensors, qos_profile=5,
        )
        self.joint_goals_pub = self.create_publisher(
            msg_type=msg.JointState, topic='joint_goals', qos_profile=5,
        )
        self.should_publish_position = Event()
        self.should_publish_velocity = Event()
        self.should_publish_effort = Event()

        self.create_timer(timer_period_sec=self.pub_period, callback=self.on_joint_goals_publish)
        self.logger.info('SDK ready to be served!')

        self.left_cam_sub = self.create_subscription(
            CompressedImage,
            'left_image',
            partial(self.decode_img, side='left'),
            1)
        self.right_cam_sub = self.create_subscription(
            CompressedImage,
            'right_image',
            partial(self.decode_img, side='right'),
            1)
        self.cv_bridge = CvBridge()
        self.cam_img = {
            'left': None,
            'right': None
        }
        self.kin_solver = OrbitaKinSolver()

    def setup(self) -> None:
        """Set up the joints values, retrieve all init info using GetJointsFullState srv."""
        self.logger.info('Getting all joints initial configuration...')

        joint_fullstate_client = self.create_client(
            srv_type=GetJointsFullState, srv_name='get_joint_full_state',
        )
        joint_fullstate_client.wait_for_service(timeout_sec=self.timeout_sec)
        fut = joint_fullstate_client.call_async(GetJointsFullState.Request())
        rclpy.spin_until_future_complete(self, fut)
        full_state_resp = fut.result()

        for i, name in enumerate(full_state_resp.name):
            pos = full_state_resp.present_position[i] if full_state_resp.present_position else None
            speed = full_state_resp.present_speed[i] if full_state_resp.present_speed else None
            load = full_state_resp.present_load[i] if full_state_resp.present_load else None

            self.joints[name] = {
                'name': name,
                'present_position': pos,
                'present_speed': speed,
                'present_load': load,
                'temperature': full_state_resp.temperature[i],
                'compliant': full_state_resp.compliant[i],
                'goal_position': full_state_resp.goal_position[i],
                'speed_limit': full_state_resp.speed_limit[i],
                'torque_limit': full_state_resp.torque_limit[i],
            }

    def on_joint_states(self, joint_state: msg.JointState) -> None:
        """Update joints position/velocity/effort on joint_state msg."""

        for i, name in enumerate(joint_state.name):
            if joint_state.position:
                self.joints[name]['present_position'] = joint_state.position[i]
            if joint_state.velocity:
                self.joints[name]['present_speed'] = joint_state.velocity[i]
            if joint_state.effort:
                self.joints[name]['present_load'] = joint_state.effort[i]

        self.joint_state_pub_event.set()

    def on_joint_temperatures(self, joint_temperature: JointTemperature) -> None:
        """Update joints temperature on joint_temperature msg."""
        for name, temp in zip(joint_temperature.name, joint_temperature.temperature):
            self.joints[name]['temperature'] = temp.temperature

    def on_load_sensors(self, load_sensor: LoadSensor) -> None:
        """Update load sensor value on load_sensor msg."""
        for side, load in zip(load_sensor.side, load_sensor.load_value):
            self.load_sensors[side] = load

    def on_joint_goals_publish(self) -> None:
        """Publish position/velocity/effort on joint_goals.

        Automatically called at a predefined frequency.
        """
        if any((
            self.should_publish_position.is_set(),
            self.should_publish_velocity.is_set(),
            self.should_publish_effort.is_set(),
        )):
            joint_goals = msg.JointState()
            joint_goals.header.stamp = self.clock.now().to_msg()
            joint_goals.name = self.joints.keys()

            if self.should_publish_position.is_set():
                joint_goals.position = [j['goal_position'] for j in self.joints.values()]
                self.should_publish_position.clear()

            if self.should_publish_velocity.is_set():
                joint_goals.velocity = [j['speed_limit'] for j in self.joints.values()]
                self.should_publish_velocity.clear()
                
            if self.should_publish_effort.is_set():
                joint_goals.effort = [j['torque_limit'] for j in self.joints.values()]
                self.should_publish_effort.clear()

            self.joint_goals_pub.publish(joint_goals)

    def handle_command(self, command: jc_pb.JointCommand) -> bool:
        """Handle new received command.

        Does not handle the async response at the moment.
        """
        name = self.id2names[command.id]

        if command.HasField('goal_position'):
            self.joints[name]['goal_position'] = command.goal_position.value
            self.should_publish_position.set()

        if command.HasField('speed_limit'):
            self.joints[name]['speed_limit'] = command.speed_limit.value
            self.should_publish_velocity.set()

        if command.HasField('torque_limit'):
            self.joints[name]['torque_limit'] = command.torque_limit.value
            self.should_publish_effort.set()

        if command.HasField('compliant'):
            request = SetCompliant.Request()
            request.name = [name]
            request.compliant = [command.compliant.value]
            future = self.compliant_client.call_async(request)
            # TODO: how to properly wait for the result and handles it?

        return True

    def decode_img(self, msg, side):
        """Callback for "/'side'_image "subscriber."""
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, img = cv.imencode('.JPEG', img)
        self.cam_img[side] = img

    # Handle GRPCs
    def GetAllJointNames(self, request: Empty, context) -> js_pb.JointNames:
        """Get all the joints name."""
        return js_pb.JointNames(names=self.joints.keys())

    def GetJointState(self, request: js_pb.JointRequest, context) -> js_pb.JointState:
        """Get the requested joint state."""
        joint = self.joints[request.name]
        fields = request.requested_fields

        return jointstate_pb_from_request(joint, fields, timestamp=True)

    def GetAllJointsState(self, request: js_pb.AllJointsRequest, context) -> js_pb.AllJointsState:
        """Get all requested joints states."""
        fields = request.requested_fields

        params = {
            'joints': [
                jointstate_pb_from_request(joint, fields, timestamp=True)
                for joint in self.joints.values()
            ],
        }
        return js_pb.AllJointsState(**params)

    def StreamAllJointsState(self, request: js_pb.StreamAllJointsRequest, context) -> Iterator[js_pb.AllJointsState]:
        """Continuously stream all joints up-to-date state."""
        dt = 1.0 / request.publish_frequency if request.publish_frequency > 0 else -1.0
        last_pub = time.time()

        fields = request.requested_fields
        timestamp = Timestamp()

        while True:
            self.joint_state_pub_event.wait()
            self.joint_state_pub_event.clear()

            timestamp.GetCurrentTime()

            params = {
                'joints': [
                    jointstate_pb_from_request(joint, fields, timestamp=False)
                    for joint in self.joints.values()
                ],
                'timestamp': timestamp,
            }
            yield js_pb.AllJointsState(**params)

            t = time.time()
            elapsed_time = t - last_pub
            if elapsed_time < dt:
                time.sleep(dt - elapsed_time)
            last_pub = t

    def SendCommand(self, request: jc_pb.JointCommand, context) -> jc_pb.JointCommandAck:
        """Handle new received command.

        Does not properly handle the async response success at the moment.
        """
        success = self.handle_command(request)
        return jc_pb.JointCommandAck(success=success)


    def SendAllJointsCommand(self, request: jc_pb.MultipleJointsCommand, context) -> jc_pb.JointCommandAck:
        success = True
        for cmd in request.commands:
            resp = self.handle_command(cmd)
            if not resp:
                success = False
        return jc_pb.JointCommandAck(success=success)


    def StreamJointsCommand(self, request_iterator: Iterator[jc_pb.MultipleJointsCommand], context) -> jc_pb.JointCommandAck:
        success = True
        for request in request_iterator:
            for cmd in request.commands:
                resp = self.handle_command(cmd)
                if not resp:
                    success = False
        return jc_pb.JointCommandAck(success=success)

    def GetLoad(self, request: ls_pb.LoadValue, context):
        """Get the requested load value."""
        load_msg = ls_pb.LoadValue()
        load_msg.load = self.load_sensors[request.side]
        return load_msg

    # Camera GRPC
    def GetImage(self, request, context):
        """Get the image from the requested camera topic."""
        im_msg = cam_pb.Image()
        im_msg.data = self.cam_img[request.side].tobytes()
        return im_msg

    # # Orbita GRPC
    # def ComputeOrbitaIK(self, request, context):
    #     """Compute Orbita's disks positions for a requested quaternion."""
    #     # tic = time.time()
    #     orb_ik_request = GetOrbitaIK.Request()
    #     orb_ik_request.quat.x = request.q.x
    #     orb_ik_request.quat.y = request.q.y
    #     orb_ik_request.quat.z = request.q.z
    #     orb_ik_request.quat.w = request.q.w
    #     future = self.orbita_ik_client.call_async(orb_ik_request)
    #     tic = time.time()
    #     while not future.done():
    #         time.sleep(0.001)
    #     self.logger.info("Compute orbita took %f" %(time.time() - tic))
    #     response = future.result()
    #     ik_msg = kin_pb.JointsPosition(
    #         positions=response.disk_pos.position.tolist(),
    #     )
    #     # self.logger.info("Compute orbita took %f" %(time.time() - tic))
    #     return ik_msg

    # Orbita GRPC
    def ComputeOrbitaIK(self, request, context):
        """Compute Orbita's disks positions for a requested quaternion."""
        tic = time.time()
        quat_solver = Quaternion()
        quat_solver.w = request.q.w
        quat_solver.x = request.q.x
        quat_solver.y = request.q.y
        quat_solver.z = request.q.z
        response = self.kin_solver.orbita_ik(quat_solver)
        ik_msg = kin_pb.JointsPosition(
            positions=response.position.tolist(),
         )
        print("Compute orbitaIk in : ", (time.time() - tic))
        return ik_msg




def main():
    """Run the Node and the gRPC server."""
    rclpy.init()

    sdk_server = ReachySDKServer(node_name='reachy_sdk_server')

    options = [
         ('grpc.max_send_message_length', 250000),  # empirical value, might be adjusted
         ('grpc.max_receive_message_length', 250000)]
    server = grpc.server(thread_pool=ThreadPoolExecutor(max_workers=30), options=options)
    joint_state_pb2_grpc.add_JointStateServiceServicer_to_server(sdk_server, server)
    joint_command_pb2_grpc.add_JointCommandServiceServicer_to_server(sdk_server, server)
    camera_reachy_pb2_grpc.add_CameraServiceServicer_to_server(sdk_server, server)
    load_sensor_pb2_grpc.add_LoadServiceServicer_to_server(sdk_server, server)
    orbita_kinematics_pb2_grpc.add_OrbitaKinematicServicer_to_server(sdk_server, server)

    server.add_insecure_port('[::]:50055')
    server.start()

    try:
        rclpy.spin(sdk_server)
    except KeyboardInterrupt:
        pass

    server.stop(grace=None)
    server.wait_for_termination()

    sdk_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
