import time
from concurrent import futures

from google.protobuf.wrappers_pb2 import FloatValue

import grpc

import rclpy
from rclpy.node import Node

from reachy_msgs.srv import SetCompliant

from reachy_sdk_api import (
    joint_command_pb2, joint_command_pb2_grpc,
    joint_state_pb2, joint_state_pb2_grpc,
)
from reachy_sdk_api.joint_state_pb2 import JointStateField

from sensor_msgs.msg import JointState


class ReachySDKServer(Node):
    def __init__(self, timeout_sec: float = 5) -> None:
        super().__init__(node_name='reachy_sdk_server')

        self.clock = self.get_clock()

        self.joint_state_subscription = self.create_subscription(
            msg_type=JointState,
            topic='joint_states',
            callback=self.on_joint_states,
            qos_profile=1,
        )

        self.joint_goals_publisher = self.create_publisher(
            msg_type=JointState,
            topic='joint_goals',
            qos_profile=5,
        )

        self.compliant_client = self.create_client(SetCompliant, 'set_compliant')
        self.compliant_client.wait_for_service(timeout_sec=timeout_sec)

        self.joints = {}

    def on_joint_states(self, joint_state: JointState) -> None:
        for name, pos in zip(joint_state.name, joint_state.position):
            self.joints[name] = {'name': name, 'position': pos}

    def get_joint_state(self, joint_name: str):
        return self.joints[joint_name]

    def set_target_position(self, joint_name: str, target_position: float):
        joint_goals = JointState()
        joint_goals.header.stamp = self.clock.now().to_msg()
        joint_goals.name = [joint_name]
        joint_goals.position = [target_position]
        self.joint_goals_publisher.publish(joint_goals)

    def set_compliant(self, joint_name: str, compliant: bool) -> bool:
        request = SetCompliant.Request()
        request.name = [joint_name]
        request.compliant = [compliant]
        future = self.compliant_client.call_async(request)
        while not future.done():
            time.sleep(0.01)
        return future.result().success


class JointStateProvider(joint_state_pb2_grpc.JointStateServiceServicer):
    STREAM_FREQ = 100

    def __init__(self, sdk_server: ReachySDKServer) -> None:
        super().__init__()
        self.sdk_server = sdk_server

    def get_joint_state(self, name: str, requested_fields) -> joint_state_pb2.JointState:
        joint = self.sdk_server.get_joint_state(name)

        params = {}

        for field in requested_fields:
            if field == JointStateField.NAME:
                params['name'] = joint['name']
            elif field == JointStateField.POSITION:
                position = FloatValue()
                position.value = joint['position']
                params['position'] = position

            # elif field == JointStateField.SPEED:
            #     params['speed'] = joint.speed

            # elif field == JointStateField.LOAD:
            #     params['load'] = joint.load

        joint_state = joint_state_pb2.JointState(**params)
        return joint_state

    def GetJointState(self, request: joint_state_pb2.JointRequest, context):
        return self.get_joint_state(request.name, request.requested_fields)

    def StreamJointState(self, request, context):
        while True:
            yield self.GetJointState(request, context)
            time.sleep(1 / JointStateProvider.STREAM_FREQ)

    def GetAllJointsState(self, request, context):
        joints = []
        for name in self.sdk_server.joints.keys():
            joints.append(self.get_joint_state(name, request.requested_fields))

        all_joints_state = joint_state_pb2.AllJointsState()
        all_joints_state.joints.extend(joints)
        return all_joints_state

    def StreamAllJointsState(self, request, context):
        while True:
            yield self.GetAllJointsState(request, context)
            time.sleep(1 / JointStateProvider.STREAM_FREQ)


class JointCommandProvider(joint_command_pb2_grpc.JointCommandServiceServicer):
    def __init__(self, sdk_server: ReachySDKServer) -> None:
        super().__init__()
        self.sdk_server = sdk_server

    def SetCompliancy(self, request, context) -> joint_command_pb2.JointCommandAck:
        success = self.sdk_server.set_compliant(request.name, request.compliant)
        return joint_command_pb2.JointCommandAck(success=success)

    def SetTargetPosition(self, request, context) -> joint_command_pb2.JointCommandAck:
        self.sdk_server.set_target_position(request.name, request.target_position)
        return joint_command_pb2.JointCommandAck(success=True)


def main():
    rclpy.init()

    sdk_server = ReachySDKServer()

    joint_state_provider = JointStateProvider(sdk_server)
    joint_command_provider = JointCommandProvider(sdk_server)

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))

    joint_state_pb2_grpc.add_JointStateServiceServicer_to_server(joint_state_provider, server)
    joint_command_pb2_grpc.add_JointCommandServiceServicer_to_server(joint_command_provider, server)

    server.add_insecure_port('[::]:50051')
    server.start()
    rclpy.spin(sdk_server)
    rclpy.shutdown()
    server.wait_for_termination()


if __name__ == '__main__':
    main()
