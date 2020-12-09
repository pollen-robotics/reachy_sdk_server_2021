import grpc

from reachy_sdk_api import (
    joint_command_pb2, joint_command_pb2_grpc,
    joint_state_pb2, joint_state_pb2_grpc,
)
from reachy_sdk_api.joint_state_pb2 import JointStateField


if __name__ == '__main__':
    with grpc.insecure_channel('localhost:50051') as channel:
        joint_state_stub = joint_state_pb2_grpc.JointStateServiceStub(channel)
        joint_command_stub = joint_command_pb2_grpc.JointCommandServiceStub(channel)

        # SINGLE REQ/REP
        req = joint_state_pb2.JointRequest(
            name='l_shoulder_pitch',
            requested_fields=[JointStateField.POSITION],
        )
        print(joint_state_stub.GetJointState(req))

        # # SINGLE STREAM
        # req = joint_state_pb2.JointRequest(
        #     name='l_shoulder_pitch',
        #     requested_fields=[JointStateField.POSITION],
        # )
        # for resp in stub.StreamJointState(req):
        #     print(resp)

        # # ALL REQ/REP
        # req = joint_state_pb2.AllJointsRequest(
        #     requested_fields=[JointStateField.NAME, JointStateField.POSITION],
        # )
        # print(stub.GetAllJointsState(req))

        # # ALL STREAM
        # req = joint_state_pb2.AllJointsRequest(
        #     requested_fields=[JointStateField.POSITION],
        # )
        # for resp in stub.StreamAllJointsState(req):
        #     print(resp)

        # Set compliant
        req = joint_command_pb2.CompliancyCommand(
            name='l_shoulder_pitch',
            compliant=False,
        )
        resp = joint_command_stub.SetCompliancy(req)
        print(resp)

        import time
        import numpy as np

        while True:
            t0 = time.time()
            pos = np.deg2rad(45 * np.sin(2 * np.pi * 1 * time.time()))
            req = joint_command_pb2.TargetPositionCommand(
                name='l_shoulder_pitch',
                target_position=pos,
            )
            resp = joint_command_stub.SetTargetPosition(req)
            t1 = time.time()
            print((t1 - t0) * 1000)
            time.sleep(0.01)
