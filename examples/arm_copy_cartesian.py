"""
Examples showing usage of gRPC functions.

The position of the right arm is read and set to the left arm.
"""
import grpc
import time

from threading import Thread

from reachy_sdk_api import joint_pb2_grpc, arm_kinematics_pb2_grpc, fullbody_cartesian_command_pb2_grpc
from joint_pb2 import JointId, JointField, JointsCommand, JointCommand, JointsStateRequest
from arm_kinematics_pb2 import ArmFKRequest, ArmEndEffector, ArmSide, ArmJointPosition, ArmIKRequest
from fullbody_cartesian_command_pb2 import FullBodyCartesianCommand
from kinematics_pb2 import JointPosition
from google.protobuf.wrappers_pb2 import BoolValue


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
        self.stub_ik = arm_kinematics_pb2_grpc.ArmKinematicsStub(self.channel)
        self.stub_cartesian = fullbody_cartesian_command_pb2_grpc.FullBodyCartesianCommandServiceStub(self.channel)

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
        pres_pos = JointsStateRequest(
            ids=self.right_arm,
            requested_fields=[JointField.PRESENT_POSITION]
        )

        while self.alive:

            state = self.stub.GetJointsState(pres_pos)

            right_arm_pos = [pos.present_position.value for pos in state.states]
            right_arm_pos = right_arm_pos[:7]

            request_right_FK = ArmFKRequest(
                arm_position=ArmJointPosition(
                    side=ArmSide.RIGHT,
                    positions=JointPosition(
                        ids=self.right_arm[:7],
                        positions=right_arm_pos,
                    )
                )
            )

            ik_right = self.stub_ik.ComputeArmFK(request_right_FK)

            new_target = ik_right.end_effector.pose
            new_target.data[7] = -new_target.data[7]
            new_target.data[1] = -new_target.data[1]
            new_target.data[4] = -new_target.data[4]
            new_target.data[6] = -new_target.data[6]
            new_target.data[9] = -new_target.data[9]

            target_left_arm = ArmEndEffector(
                side=ArmSide.LEFT,
                pose=new_target,
            )

            left_arm_IK_request = ArmIKRequest(
                target=target_left_arm,
            )

            cartesian_command_left = FullBodyCartesianCommand(
                left_arm=left_arm_IK_request,
            )

            self.stub_cartesian.SendFullBodyCartesianCommands(cartesian_command_left)

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
