"""Expose main Reachy ROS services/topics through gRPC allowing remote client SDK."""

import time
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, Iterator
from threading import Event

from google.protobuf.empty_pb2 import Empty
from google.protobuf.timestamp_pb2 import Timestamp

import grpc

import rclpy
from rclpy.node import Node

from reachy_msgs.msg import JointTemperature
from reachy_msgs.srv import GetJointsFullState, SetCompliant

from reachy_sdk_api import joint_command_pb2 as jc_pb, joint_command_pb2_grpc
from reachy_sdk_api import joint_state_pb2 as js_pb, joint_state_pb2_grpc

from sensor_msgs import msg

from .utils import jointstate_pb_from_request


class ReachySDKServer(Node,
                      joint_state_pb2_grpc.JointStateServiceServicer,
                      joint_command_pb2_grpc.JointCommandServiceServicer):
    """Reachy SDK server node."""

    def __init__(self, node_name: str, timeout_sec: float = 5, pub_frequency: float = 100) -> None:
        """Set up the node.

        Subscribe to /joint_state, /joint_temp.
        Publish new command on /joint_goal or concerned services.

        """
        super().__init__(node_name=node_name)

        self.timeout_sec = timeout_sec
        self.pub_period = 1 / pub_frequency

        self.clock = self.get_clock()
        self.logger = self.get_logger()

        self.joints: Dict[str, Dict[str, float]] = OrderedDict()
        self.setup()
        self.id2names = {i: name for i, name in enumerate(self.joints.keys())}

        self.logger.info('Launching pub/sub/srv...')
        self.compliant_client = self.create_client(SetCompliant, 'set_compliant')

        self.joint_states_sub = self.create_subscription(
            msg_type=msg.JointState, topic='joint_states',
            callback=self.on_joint_states, qos_profile=5,
        )
        self.joint_state_pub_event = Event()
        self.joint_temperatures_sub = self.create_subscription(
            msg_type=JointTemperature, topic='joint_temperatures',
            callback=self.on_joint_temperatures, qos_profile=5,
        )

        self.joint_goals_pub = self.create_publisher(
            msg_type=msg.JointState, topic='joint_goals', qos_profile=5,
        )
        self.create_timer(timer_period_sec=self.pub_period, callback=self.on_joint_goals_publish)
        self.logger.info('SDK ready to be served!')

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

    def on_joint_goals_publish(self) -> None:
        """Publish position/velocity/effort on joint_goals.

        Automatically called at a predefined frequency.
        """
        # TODO: only publish when and what's needed?
        joint_goals = msg.JointState()
        joint_goals.header.stamp = self.clock.now().to_msg()
        joint_goals.name = self.joints.keys()
        joint_goals.position = [j['goal_position'] for j in self.joints.values()]
        joint_goals.velocity = [j['speed_limit'] for j in self.joints.values()]
        joint_goals.effort = [j['torque_limit'] for j in self.joints.values()]
        self.joint_goals_pub.publish(joint_goals)

    def handle_command(self, command: jc_pb.JointCommand) -> bool:
        """Handle new received command.

        Does not handle the async response at the moment.
        """
        name = self.id2names[command.id]

        if command.HasField('goal_position'):
            self.joints[name]['goal_position'] = command.goal_position.value

        if command.HasField('speed_limit'):
            self.joints[name]['speed_limit'] = command.speed_limit.value

        if command.HasField('torque_limit'):
            self.joints[name]['torque_limit'] = command.torque_limit.value

        if command.HasField('compliant'):
            request = SetCompliant.Request()
            request.name = [name]
            request.compliant = [command.compliant.value]
            future = self.compliant_client.call_async(request)
            # TODO: how to properly wait for the result and handles it?

        return True

    # Handle GRPCs
    def GetAllJointNames(self, request: Empty, context) -> js_pb.JointNames:
        """Get all the joints name."""
        return js_pb.JointNames(names=self.joints.keys())

    def GetJointState(self, request: js_pb.JointRequest, context) -> js_pb.JointState:
        """Get the requested joint state."""
        joint = self.joints[request.name]
        fields = request.requested_fields

        return jointstate_pb_from_request(joint, fields, timestamp=True)

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

    def StreamJointsCommand(self, request_iterator: Iterator[jc_pb.MultipleJointsCommand], context) -> jc_pb.JointCommandAck:
        success = True
        for request in request_iterator:
            for cmd in request.commands:
                resp = self.handle_command(cmd)
                if not resp:
                    success = False
        return jc_pb.JointCommandAck(success=success)


def main():
    """Run the Node and the gRPC server."""
    rclpy.init()

    sdk_server = ReachySDKServer(node_name='reachy_sdk_server')

    server = grpc.server(thread_pool=ThreadPoolExecutor(max_workers=10))
    joint_state_pb2_grpc.add_JointStateServiceServicer_to_server(sdk_server, server)
    joint_command_pb2_grpc.add_JointCommandServiceServicer_to_server(sdk_server, server)

    server.add_insecure_port('[::]:50051')
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
