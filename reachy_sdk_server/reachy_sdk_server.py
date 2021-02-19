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
from reachy_msgs.srv import GetArmIK, GetArmFK, GetOrbitaIK
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


class ReachySDKServer(Node,
                      joint_state_pb2_grpc.JointStateServiceServicer,
                      joint_command_pb2_grpc.JointCommandServiceServicer,
                      camera_reachy_pb2_grpc.CameraServiceServicer,
                      load_sensor_pb2_grpc.LoadServiceServicer,
                      orbita_kinematics_pb2_grpc.OrbitaKinematicServicer,
                      arm_kinematics_pb2_grpc.ArmKinematicServicer,
                      cartesian_command_pb2_grpc.CartesianCommandServiceServicer,
                      zoom_command_pb2_grpc.ZoomControllerServiceServicer,
                      ):
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
        self.zoom_command_client = self.create_client(ZoomCommand, 'zoom_command')
        self.zoom_speed_client = self.create_client(SetZoomSpeed, 'zoom_speed')

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
        self.cam_img = {
            'left': None,
            'right': None
        }

        # Kinematics
        self.left_arm_fk = self.create_client(GetArmFK, '/reachy_left_arm_kinematics_service/forward')
        self.left_arm_ik = self.create_client(GetArmIK, '/reachy_left_arm_kinematics_service/inverse')
        self.right_arm_fk = self.create_client(GetArmFK, '/reachy_right_arm_kinematics_service/forward')
        self.right_arm_ik = self.create_client(GetArmIK, '/reachy_right_arm_kinematics_service/inverse')
        self.orbita_ik = self.create_client(GetOrbitaIK, '/orbita_ik')

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

    def handle_commands(self, commands: List[jc_pb.JointCommand]) -> bool:
        """Handle new received commands."""
        success = True

        # Handles first compliancy as it requires specific service call.
        names, values = [], []
        for cmd in commands:
            if cmd.HasField('compliant'):
                names.append(self.id2names[cmd.id])
                values.append(cmd.compliant.value)
        if names:
            request = SetCompliant.Request()
            request.name = names
            request.compliant = values

            # TODO: Should be re-written using asyncio
            future = self.compliant_client.call_async(request)
            for _ in range(100):
                if future.done():
                    success = future.result()
                time.sleep(0.01)
            else:
                success = False

        use_goal_pos, use_goal_vel, use_goal_eff = False, False, False
        for cmd in commands:
            name = self.id2names[cmd.id]

            if cmd.HasField('goal_position'):
                self.joints[name]['goal_position'] = cmd.goal_position.value
                use_goal_pos = True

            if cmd.HasField('speed_limit'):
                self.joints[name]['speed_limit'] = cmd.speed_limit.value
                use_goal_vel = True

            if cmd.HasField('torque_limit'):
                self.joints[name]['torque_limit'] = cmd.torque_limit.value
                use_goal_eff = True

        if use_goal_pos:
            self.should_publish_position.set()
        if use_goal_vel:
            self.should_publish_velocity.set()
        if use_goal_eff:
            self.should_publish_effort.set()

        return success

    def decode_img(self, msg, side):
        """Callback for "/'side'_image "subscriber."""
        self.cam_img[side] = msg.data

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
        success = self.handle_commands([request])
        return jc_pb.JointCommandAck(success=success)

    def SendAllJointsCommand(self, request: jc_pb.MultipleJointsCommand, context) -> jc_pb.JointCommandAck:
        success = self.handle_commands(request.commands)
        return jc_pb.JointCommandAck(success=success)

    def StreamJointsCommand(self, request_iterator: Iterator[jc_pb.MultipleJointsCommand], context) -> jc_pb.JointCommandAck:
        success = True
        for request in request_iterator:
            resp = self.handle_commands(request.commands)
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

    # Orbita GRPC
    def ComputeOrbitaIK(self, request: orbita_pb.OrbitaTarget, context) -> kin_pb.JointsPosition:
        """Compute Orbita's disks positions for a requested quaternion."""
        ros_req = GetOrbitaIK.Request()
        ros_req.quat = Quaternion(
            x=request.q.x,
            y=request.q.y,
            z=request.q.z,
            w=request.q.w,
        )
        resp = self.orbita_ik.call(ros_req)

        return kin_pb.JointsPosition(
            positions=resp.disk_pos.position,
        )

    # Arm kinematics GRPC
    def ComputeArmFK(self, request: armk_pb.ArmJointsPosition, context) -> armk_pb.ArmEndEffector:
        """Compute forward kinematics for requested arm."""
        fk_client = self.left_arm_fk if request.side == armk_pb.ArmSide.LEFT else self.right_arm_fk

        req = GetArmFK.Request()
        req.joint_position.position = request.positions.positions

        resp = fk_client.call(req)
        M = np.eye(4)

        p = resp.pose.position
        M[:3, 3] = p.x, p.y, p.z

        q = resp.pose.orientation
        M[:3, :3] = Rotation.from_quat((q.x, q.y, q.z, q.w)).as_matrix()

        return armk_pb.ArmEndEffector(
            side=request.side,
            target=kin_pb.Matrix4x4(data=M.flatten()),
        )

    def _call_arm_ik(self, request: armk_pb.ArmEndEffector):
        ik_client = self.left_arm_ik if request.side == armk_pb.ArmSide.LEFT else self.right_arm_ik

        ros_req = GetArmIK.Request()
        M = np.array(request.target.data).reshape((4, 4))

        ros_req.pose.position = Point(x=M[0, 3], y=M[1, 3], z=M[2, 3])
        q = Rotation.from_matrix(M[:3, :3]).as_quat()
        ros_req.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        if request.q0:
            ros_req.q0.position = request.q0.positions

        return ik_client.call(ros_req)

    def ComputeArmIK(self, request: armk_pb.ArmEndEffector, context) -> armk_pb.ArmJointsPosition:
        """Compute inverse kinematics for requested arm."""
        resp = self._call_arm_ik(request)

        return armk_pb.ArmJointsPosition(
            side=request.side,
            positions=kin_pb.JointsPosition(positions=resp.joint_position.position),
        )

    def SendCartesianCommand(self, request: cart_pb.FullBodyCartesianCommand, context) -> cart_pb.CartesianCommandAck:
        goal_position = {}

        if request.HasField('left_arm_end_effector'):
            request.left_arm_end_effector.side = armk_pb.ArmSide.LEFT
            resp = self._call_arm_ik(request.left_arm_end_effector)
            goal_position.update(dict(zip(resp.joint_position.name, resp.joint_position.position)))

        if request.HasField('right_arm_end_effector'):
            request.right_arm_end_effector.side = armk_pb.ArmSide.RIGHT
            resp = self._call_arm_ik(request.right_arm_end_effector)
            goal_position.update(dict(zip(resp.joint_position.name, resp.joint_position.position)))

        if request.HasField('orbita_target'):
            resp = self.ComputeOrbitaIK(request.orbita_target, context)
            disks = ['neck_disk_top', 'neck_disk_middle', 'neck_disk_bottom']
            goal_position.update(dict(zip(disks, resp.positions)))

        for name, pos in goal_position.items():
            self.joints[name]['goal_position'] = pos
        self.should_publish_position.set()

        return cart_pb.CartesianCommandAck(success=True)

    def StreamCartesianCommands(self, request_iterator: cart_pb.FullBodyCartesianCommand, context) -> cart_pb.CartesianCommandAck:
        for request in request_iterator:
            self.SendCartesianCommand(request, context)
        return cart_pb.CartesianCommandAck(success=True)

    # ZoomController GRPC
    def SendZoomCommand(self, request, context):
        """Send command to zoom controller of the requested camera."""
        req = ZoomCommand.Request()
        req.side = request.side
        req.zoom_command = request.command
        future = self.zoom_command_client.call_async(req)
        return zc_pb.Empty()

    def SetZoomSpeed(self, request, context):
        """Change zoom controller motors speed."""
        req = SetZoomSpeed.Request()
        req.speed = request.speed
        future = self.zoom_speed_client.call_async(req)
        return zc_pb.Empty()


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
    arm_kinematics_pb2_grpc.add_ArmKinematicServicer_to_server(sdk_server, server)
    zoom_command_pb2_grpc.add_ZoomControllerServiceServicer_to_server(sdk_server, server)
    cartesian_command_pb2_grpc.add_CartesianCommandServiceServicer_to_server(sdk_server, server)

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
