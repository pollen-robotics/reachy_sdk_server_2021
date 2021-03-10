"""
Examples showing usage of gRPC functions.

The position of the right arm is read and set to the left arm.
"""
import grpc
import time

from threading import Thread

from reachy_sdk_api import joint_pb2_grpc
from joint_pb2 import JointId, JointField, JointsCommand, JointCommand, JointsStateRequest
from google.protobuf.wrappers_pb2 import FloatValue, BoolValue


class ArmCopyGRPC():
    """Class enabling to copy movements applied to the right arm on the left one."""

    left_arm = [
        JointId(name='l_shoulder_pitch'),
        JointId(name='l_shoulder_roll'),
        JointId(name='l_arm_yaw'),
        JointId(name='l_elbow_pitch'),
        JointId(name='l_forearm_yaw'),
        JointId(name='l_wrist_pitch'),
        JointId(name='l_wrist_roll'),
        JointId(name='l_gripper'),
    ]
    right_arm = [
        JointId(name='r_shoulder_pitch'),
        JointId(name='r_shoulder_roll'),
        JointId(name='r_arm_yaw'),
        JointId(name='r_elbow_pitch'),
        JointId(name='r_forearm_yaw'),
        JointId(name='r_wrist_pitch'),
        JointId(name='r_wrist_roll'),
        JointId(name='r_gripper'),
    ]

    def __init__(self) -> None:
        """Set up the class and open connection to server and service."""
        self.channel = grpc.insecure_channel('localhost:50055')
        self.stub = joint_pb2_grpc.JointServiceStub(self.channel)

        self.alive = True

    def set_compliancy_left(self, compliancy) -> None:
        """Set left arm stiff or compliant."""
        request = JointsCommand(
            commands=[
                JointCommand(id=name, compliant=BoolValue(value=compliancy))
                for name in self.left_arm
            ]
        )
        self.stub.SendJointsCommands(request)

    def get_pose(self):
        """
        Handle reproduction of right arm movements on the left arm.

        Read right arm joints state and apply the modified commands on the left arm.
        Callback for the class thread.
        """
        request = JointsStateRequest(
            ids=self.right_arm,
            requested_fields=[JointField.PRESENT_POSITION]
        )
        while self.alive:
            state = self.stub.GetJointsState(request)

            left_pos = [-pos.present_position.value for pos in state.states]
            for i, pos in enumerate(left_pos):
                if (i == 0 or i == 3 or i == 5):
                    left_pos[i] = -pos

            command = JointsCommand(
                commands=[
                    JointCommand(id=name, goal_position=FloatValue(value=pos))
                    for name, pos in zip(self.left_arm, left_pos)
                ]
            )

            self.stub.SendJointsCommands(command)
            time.sleep(0.01)

    def start(self):
        """Start thread for arm copy."""
        self.set_compliancy_left(False)
        self.t = Thread(target=self.get_pose)
        self.t.start()

    def stop(self):
        """Stop thread and set left arm compliant."""
        self.alive = False
        self.t.join()
        self.set_compliancy_left(True)


if __name__ == '__main__':
    arm_copy = ArmCopyGRPC()

    time.sleep(0.01)
    arm_copy.start()
    input("Press 'Enter' to stop example...")

    arm_copy.stop()
    time.sleep(0.01)

    print('Example stopped.')
