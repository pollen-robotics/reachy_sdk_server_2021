"""Expose main Reachy ROS services/topics through gRPC allowing remote client SDK."""

import threading
import time

from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, Iterator, List

import numpy as np

from scipy.spatial.transform import Rotation

from google.protobuf.empty_pb2 import Empty
from google.protobuf.timestamp_pb2 import Timestamp

import grpc

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import JointState

from reachy_msgs.msg import JointTemperature, ForceSensor, PidGains
from reachy_msgs.srv import GetJointFullState, SetJointCompliancy, SetJointPidGains
from reachy_msgs.srv import GetArmIK, GetArmFK, GetOrbitaIK, GetQuaternionTransform

from reachy_sdk_api import joint_pb2, joint_pb2_grpc
from reachy_sdk_api import sensor_pb2, sensor_pb2_grpc
from reachy_sdk_api import orbita_kinematics_pb2, orbita_kinematics_pb2_grpc
from reachy_sdk_api import kinematics_pb2
from reachy_sdk_api import arm_kinematics_pb2, arm_kinematics_pb2_grpc
from reachy_sdk_api import fullbody_cartesian_command_pb2, fullbody_cartesian_command_pb2_grpc


from .utils import jointstate_pb_from_request


proto_arm_side_to_str = {
    arm_kinematics_pb2.ArmSide.LEFT: 'left',
    arm_kinematics_pb2.ArmSide.RIGHT: 'right',
}


class ReachySDKServer(Node,
                      joint_pb2_grpc.JointServiceServicer,
                      sensor_pb2_grpc.SensorServiceServicer,
                      orbita_kinematics_pb2_grpc.OrbitaKinematicsServicer,
                      arm_kinematics_pb2_grpc.ArmKinematicsServicer,
                      fullbody_cartesian_command_pb2_grpc.FullBodyCartesianCommandServiceServicer,
                      ):
    """Reachy SDK server node."""

    def __init__(self, node_name: str, timeout_sec: float = 5, pub_frequency: float = 100) -> None:
        """Set up the node.

        Subscribe to /joint_states, /joint_temperatures, /force_sensors.
        Publish new command on /joint_goals or concerned services.

        """
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()

        self.clock = self.get_clock()

        self.timeout_sec = timeout_sec
        self.pub_period = 1 / pub_frequency

        self.joints: Dict[str, Dict[str, float]] = OrderedDict()
        self.force_sensors: Dict[str, float] = OrderedDict()
        self.setup()

        self.id2names = {i: name for i, name in enumerate(self.joints.keys())}
        self.names2ids = {name: i for i, name in enumerate(self.joints.keys())}
        for name, uid in self.names2ids.items():
            self.joints[name]['uid'] = uid

        self.logger.info('Launching pub/sub/srv...')
        self.compliant_client = self.create_client(SetJointCompliancy, 'set_joint_compliancy')

        self.set_pid_client = self.create_client(SetJointPidGains, 'set_joint_pid')

        self.joint_states_pub_event = threading.Event()
        self.joint_states_sub = self.create_subscription(
            msg_type=JointState, topic='joint_states',
            callback=self.on_joint_states, qos_profile=5,
        )

        self.joint_temperatures_sub = self.create_subscription(
            msg_type=JointTemperature, topic='joint_temperatures',
            callback=self.on_joint_temperatures, qos_profile=5,
        )

        self.force_sensor_sub = self.create_subscription(
            msg_type=ForceSensor, topic='force_sensors',
            callback=self.on_force_sensors, qos_profile=5,
        )

        self.joint_goals_pub = self.create_publisher(
            msg_type=JointState, topic='joint_goals', qos_profile=5,
        )
        self.should_publish_position = threading.Event()
        self.should_publish_velocity = threading.Event()
        self.should_publish_effort = threading.Event()
        self.create_timer(timer_period_sec=self.pub_period, callback=self.on_joint_goals_publish)

        # Kinematics
        self.left_arm_fk = self.create_client(GetArmFK, '/left_arm/kinematics/forward')
        self.left_arm_ik = self.create_client(GetArmIK, '/left_arm/kinematics/inverse')
        self.right_arm_fk = self.create_client(GetArmFK, '/right_arm/kinematics/forward')
        self.right_arm_ik = self.create_client(GetArmIK, '/right_arm/kinematics/inverse')
        self.orbita_ik = self.create_client(GetOrbitaIK, '/orbita/kinematics/inverse')
        self.orbita_look_at_tf = self.create_client(GetQuaternionTransform, '/orbita/kinematics/look_vector_to_quaternion')

        self.logger.info('SDK ready to be served!')

    def _repr_ros_pid(self, pid: PidGains) -> List:
        if pid.p != 0:
            return [pid.p, pid.i, pid.d]
        else:
            return [pid.cw_compliance_margin, pid.ccw_compliance_margin, pid.cw_compliance_slope, pid.ccw_compliance_slope]

    def _repr_proto_pid(self, pid: joint_pb2.PIDValue) -> List:
        if pid.HasField('pid'):
            return [pid.pid.p, pid.pid.i, pid.pid.d]
        else:
            return [pid.compliance.cw_compliance_margin, pid.compliance.ccw_compliance_margin,
                    pid.compliance.cw_compliance_slope, pid.compliance.ccw_compliance_slope]

    def setup(self) -> None:
        """Set up the joints values, retrieve all init info using GetJointFullState srv."""
        self.logger.info('Getting all joints initial configuration...')

        joint_fullstate_client = self.create_client(
            srv_type=GetJointFullState, srv_name='get_joint_full_state',
        )
        joint_fullstate_client.wait_for_service(timeout_sec=self.timeout_sec)
        fut = joint_fullstate_client.call_async(GetJointFullState.Request())
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
                'pid': self._repr_ros_pid(full_state_resp.pid_gain[i]),
            }

    def on_joint_states(self, joint_state: JointState) -> None:
        """Update joints position/velocity/effort on joint_state msg."""
        for i, name in enumerate(joint_state.name):
            if joint_state.position:
                self.joints[name]['present_position'] = joint_state.position[i]
            if joint_state.velocity:
                self.joints[name]['present_speed'] = joint_state.velocity[i]
            if joint_state.effort:
                self.joints[name]['present_load'] = joint_state.effort[i]

        self.joint_states_pub_event.set()

    def on_joint_temperatures(self, joint_temperature: JointTemperature) -> None:
        """Update joints temperature on joint_temperature msg."""
        for name, temp in zip(joint_temperature.name, joint_temperature.temperature):
            self.joints[name]['temperature'] = temp.temperature

    def on_force_sensors(self, force_sensor: ForceSensor) -> None:
        """Update load sensor value on load_sensor msg."""
        for name, force in zip(force_sensor.name, force_sensor.force):
            self.force_sensors[name] = force

    def on_joint_goals_publish(self) -> None:
        """Publish position/velocity/effort on joint_goals.

        Automatically called at a predefined frequency.
        """
        if any((
            self.should_publish_position.is_set(),
            self.should_publish_velocity.is_set(),
            self.should_publish_effort.is_set(),
        )):
            joint_goals = JointState()
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

    def _joint_id_to_name(self, joint_id: joint_pb2.JointId) -> str:
        if joint_id.HasField('name'):
            return joint_id.name
        elif joint_id.HasField('uid'):
            return self.id2names[joint_id.uid]
        else:
            raise ValueError(f'Unknown joint_id {joint_id}!')

    def handle_commands(self, commands: List[joint_pb2.JointCommand]) -> bool:
        """Handle new received commands."""
        success = True

        # Handles first compliancy as it requires specific service call.
        names, values = [], []
        for cmd in commands:
            if cmd.HasField('compliant'):
                names.append(self._joint_id_to_name(cmd.id))
                values.append(cmd.compliant.value)
        if names:
            request = SetJointCompliancy.Request()
            request.name = names
            request.compliancy = values

            # TODO: Should be re-written using asyncio
            future = self.compliant_client.call_async(request)
            for _ in range(1000):
                if future.done():
                    success = future.result().success
                    break
                time.sleep(0.001)
            else:
                success = False

        names_pid, pid_gains = [], []
        for cmd in commands:
            if not cmd.HasField('pid'):
                continue
            name = self._joint_id_to_name(cmd.id)
            if cmd.pid.HasField('pid'):
                pid_gain = PidGains(
                    p=cmd.pid.pid.p,
                    i=cmd.pid.pid.i,
                    d=cmd.pid.pid.d,
                    )
            elif cmd.pid.HasField('compliance'):
                pid_gain = PidGains(
                    cw_compliance_margin=cmd.pid.compliance.cw_compliance_margin,
                    ccw_compliance_margin=cmd.pid.compliance.ccw_compliance_margin,
                    cw_compliance_slope=cmd.pid.compliance.cw_compliance_slope,
                    ccw_compliance_slope=cmd.pid.compliance.ccw_compliance_slope,
                )
            names_pid.append(name)
            pid_gains.append(pid_gain)
        if names_pid:
            request = SetJointPidGains.Request(
                name=names_pid,
                pid_gain=pid_gains,
                )
            future = self.set_pid_client.call_async(request)
            # TODO: Should be re-written using asyncio
            for _ in range(1000):
                if future.done():
                    success = future.result().success
                    break
                time.sleep(0.001)
            else:
                success = False

        use_goal_pos, use_goal_vel, use_goal_eff = False, False, False
        for cmd in commands:
            name = self._joint_id_to_name(cmd.id)

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

    # Handle GRPCs
    # Joint Service
    def GetAllJointsId(self, request: Empty, context) -> joint_pb2.JointsId:
        """Get all the joints name."""
        uids, names = zip(*enumerate(self.joints.keys()))
        return joint_pb2.JointsId(names=names, uids=uids)

    def GetJointsState(self, request: joint_pb2.JointsStateRequest, context) -> joint_pb2.JointsState:
        """Get the requested joints id."""
        params = {}

        params['ids'] = request.ids
        params['states'] = [
            jointstate_pb_from_request(
                self.joints[self._joint_id_to_name(id)],
                request.requested_fields,
            )
            for id in request.ids
        ]
        params['timestamp'] = Timestamp()
        params['timestamp'].GetCurrentTime()

        return joint_pb2.JointsState(**params)

    def StreamJointsState(self, request: joint_pb2.StreamJointsRequest, context) -> Iterator[joint_pb2.JointsState]:
        """Continuously stream requested joints up-to-date state."""
        dt = 1.0 / request.publish_frequency if request.publish_frequency > 0 else -1.0
        last_pub = 0.0

        while True:
            elapsed_time = time.time() - last_pub
            if elapsed_time < dt:
                time.sleep(dt - elapsed_time)

            self.joint_states_pub_event.wait()
            self.joint_states_pub_event.clear()

            joints_state = self.GetJointsState(request.request, context)
            joints_state.timestamp.GetCurrentTime()

            yield joints_state
            last_pub = time.time()

    def SendJointsCommands(self, request: joint_pb2.JointsCommand, context) -> joint_pb2.JointsCommandAck:
        """Handle new received commands.

        Does not properly handle the async response success at the moment.
        """
        success = self.handle_commands(request.commands)
        return joint_pb2.JointsCommandAck(success=success)

    def StreamJointsCommands(self, request_iterator: Iterator[joint_pb2.JointsCommand], context) -> joint_pb2.JointsCommandAck:
        """Handle stream of commands for multiple joints."""
        success = True
        for request in request_iterator:
            resp = self.handle_commands(request.commands)
            if not resp:
                success = False
        return joint_pb2.JointsCommandAck(success=success)

    # Sensor Service
    def GetAllForceSensorsId(self, request: Empty, context) -> sensor_pb2.SensorsId:
        """Get all the force sensors id."""
        names, uids = zip(*enumerate(self.force_sensors.keys()))
        return sensor_pb2.SensorsId(names=names, uids=uids)

    def GetSensorsState(self, request: sensor_pb2.SensorsStateRequest, context) -> sensor_pb2.SensorsState:
        """Get the requested sensors value."""
        forces = list(self.force_sensors.values())

        params = {}
        params['ids'] = request.ids
        params['states'] = [
            sensor_pb2.SensorState(force_sensor_state=sensor_pb2.ForceSensorState(force=forces[id]))
            for id in request.ids
        ]

        return sensor_pb2.SensorState(**params)

    def StreamSensorStates(self, request: sensor_pb2.StreamSensorsStateRequest, context) -> Iterator[sensor_pb2.SensorsState]:
        """Continuously stream requested sensors up-to-date value."""
        dt = 1.0 / request.publish_frequency if request.publish_frequency > 0 else -1.0
        last_pub = 0.0

        while True:
            elapsed_time = time.time() - last_pub
            if elapsed_time < dt:
                time.sleep(dt - elapsed_time)

            sensors_state = self.GetSensorsState(request.request, context)
            sensors_state.timestamp.GetCurrentTime()

            yield sensors_state
            last_pub = time.time()

    # Kinematics Service
    def ComputeOrbitaIK(
        self,
        request: orbita_kinematics_pb2.OrbitaIKRequest,
        context,
    ) -> orbita_kinematics_pb2.OrbitaIKSolution:
        """Compute Orbita's disks positions for a requested quaternion."""
        ros_req = GetOrbitaIK.Request()
        ros_req.orientation = Quaternion(
            x=request.q.x,
            y=request.q.y,
            z=request.q.z,
            w=request.q.w,
        )
        resp = self.orbita_ik.call(ros_req)
        if not resp.success:
            return orbita_kinematics_pb2.OrbitaIKSolution(success=False)

        return orbita_kinematics_pb2.OrbitaIKSolution(
            success=True,
            disk_position=kinematics_pb2.JointPosition(
                ids=[joint_pb2.JointId(uid=self.names2ids[f'neck_{name}']) for name in resp.disk_position.name],
                positions=resp.disk_position.position,
            ),
        )

    def GetQuaternionTransform(self, request: orbita_kinematics_pb2.LookVector, context) -> kinematics_pb2.Quaternion:
        """Get quaternion from the given look at vector."""
        ros_req = GetQuaternionTransform.Request()
        ros_req.look_vector = Vector3(
            x=request.x,
            y=request.y,
            z=request.z,
        )
        resp = self.orbita_look_at_tf.call(ros_req)
        return kinematics_pb2.Quaternion(
            w=resp.orientation.w,
            x=resp.orientation.x,
            y=resp.orientation.y,
            z=resp.orientation.z,
        )

    def ComputeArmFK(self, request: arm_kinematics_pb2.ArmFKRequest, context) -> arm_kinematics_pb2.ArmFKSolution:
        """Compute forward kinematics for requested arm."""
        fk_client = self.left_arm_fk if request.arm_position.side == arm_kinematics_pb2.ArmSide.LEFT else self.right_arm_fk

        req = GetArmFK.Request()
        req.joint_position.position = request.arm_position.positions.positions

        resp = fk_client.call(req)
        M = np.eye(4)

        p = resp.pose.position
        M[:3, 3] = p.x, p.y, p.z

        q = resp.pose.orientation
        M[:3, :3] = Rotation.from_quat((q.x, q.y, q.z, q.w)).as_matrix()

        return arm_kinematics_pb2.ArmFKSolution(
            success=True,
            end_effector=arm_kinematics_pb2.ArmEndEffector(
                side=request.arm_position.side,
                pose=kinematics_pb2.Matrix4x4(data=M.flatten()),
            ),
        )

    def _call_arm_ik(self, request: arm_kinematics_pb2.ArmIKRequest) -> GetArmIK.Response:
        ik_client = self.left_arm_ik if request.target.side == arm_kinematics_pb2.ArmSide.LEFT else self.right_arm_ik

        ros_req = GetArmIK.Request()
        M = np.array(request.target.pose.data).reshape((4, 4))

        ros_req.pose.position = Point(x=M[0, 3], y=M[1, 3], z=M[2, 3])
        q = Rotation.from_matrix(M[:3, :3]).as_quat()
        ros_req.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        if request.q0:
            ros_req.q0.position = request.q0.positions

        return ik_client.call(ros_req)

    def ComputeArmIK(self, request: arm_kinematics_pb2.ArmIKRequest, context) -> arm_kinematics_pb2.ArmIKSolution:
        """Compute inverse kinematics for requested arm."""
        resp = self._call_arm_ik(request)

        if not resp.success:
            return arm_kinematics_pb2.ArmIKSolution(success=False)

        return arm_kinematics_pb2.ArmIKSolution(
            success=True,
            arm_position=arm_kinematics_pb2.ArmJointPosition(
                side=request.target.side,
                positions=kinematics_pb2.JointPosition(
                    ids=[joint_pb2.JointId(uid=self.names2ids[name]) for name in resp.joint_position.name],
                    positions=resp.joint_position.position,
                ),
            ),
        )

    def SendFullBodyCartesianCommands(
        self,
        request: fullbody_cartesian_command_pb2.FullBodyCartesianCommand,
        context,
    ) -> fullbody_cartesian_command_pb2.FullBodyCartesianCommandAck:
        """Compute movement given the requested commands in cartesian space."""
        left_arm_success = True
        right_arm_success = True
        orbita_head_success = True

        running = threading.Event()

        def bg():
            running.set()

            goal_position = {}

            if request.HasField('left_arm'):
                resp = self._call_arm_ik(request.left_arm)
                if resp.success:
                    goal_position.update(dict(zip(
                        resp.joint_position.name,
                        resp.joint_position.position,
                    )))
                else:
                    nonlocal left_arm_success
                    left_arm_success = False

            if request.HasField('right_arm'):
                resp = self._call_arm_ik(request.right_arm)
                if resp.success:
                    goal_position.update(dict(zip(
                        resp.joint_position.name,
                        resp.joint_position.position,
                    )))
                else:
                    nonlocal right_arm_success
                    right_arm_success = False

            if request.HasField('neck'):
                resp = self.ComputeOrbitaIK(request.neck, context)
                if resp.success:
                    goal_position.update(dict(zip(
                        (self._joint_id_to_name(name) for name in resp.disk_position.ids),
                        resp.disk_position.positions,
                    )))
                else:
                    nonlocal orbita_head_success
                    orbita_head_success = False

            for name, pos in goal_position.items():
                self.joints[name]['goal_position'] = pos
            self.should_publish_position.set()

        t = threading.Thread(target=bg)
        t.daemon = True
        t.start()

        running.wait()

        for _ in range(100):
            if not t.is_alive():
                break
            time.sleep(0.001)
        else:
            self.logger.warning('ik service timeout!')
            left_arm_success = False
            right_arm_success = False
            orbita_head_success = False

        return fullbody_cartesian_command_pb2.FullBodyCartesianCommandAck(
            left_arm_command_success=left_arm_success,
            right_arm_command_success=right_arm_success,
            neck_command_success=orbita_head_success,
        )

    def StreamFullBodyCartesianCommands(
        self,
        request_iterator: Iterator[fullbody_cartesian_command_pb2.FullBodyCartesianCommand],
        context,
    ) -> fullbody_cartesian_command_pb2.FullBodyCartesianCommandAck:
        """Compute movement from stream of commands in cartesian space."""
        for request in request_iterator:
            _ = self.SendCartesianCommand(request, context)
        return fullbody_cartesian_command_pb2.FullBodyCartesianCommandAck(
            left_arm_command_success=True,
            right_arm_command_success=True,
            neck_command_success=True,
        )


def main():
    """Run the Node and the gRPC server."""
    rclpy.init()

    sdk_server = ReachySDKServer(node_name='reachy_sdk_server')

    server = grpc.server(thread_pool=ThreadPoolExecutor(max_workers=10))
    joint_pb2_grpc.add_JointServiceServicer_to_server(sdk_server, server)
    sensor_pb2_grpc.add_SensorServiceServicer_to_server(sdk_server, server)
    orbita_kinematics_pb2_grpc.add_OrbitaKinematicsServicer_to_server(sdk_server, server)
    arm_kinematics_pb2_grpc.add_ArmKinematicsServicer_to_server(sdk_server, server)
    fullbody_cartesian_command_pb2_grpc.add_FullBodyCartesianCommandServiceServicer_to_server(sdk_server, server)

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
