"""Expose main Reachy ROS services/topics through gRPC allowing remote client SDK."""

import threading
import time
from subprocess import check_output

from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, Iterator, List

import numpy as np

from scipy.spatial.transform import Rotation

from google.protobuf.empty_pb2 import Empty
from google.protobuf.timestamp_pb2 import Timestamp
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue

import grpc

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Quaternion

from sensor_msgs.msg import JointState

from reachy_msgs.msg import JointTemperature, ForceSensor, PidGains, FanState
from reachy_msgs.srv import GetJointFullState, SetJointCompliancy, SetJointPidGains
from reachy_msgs.srv import GetArmIK, GetArmFK
from reachy_msgs.srv import GetReachyModel, SetFanState
from reachy_msgs.msg import Gripper as GripperMsg

from reachy_sdk_api import joint_pb2, joint_pb2_grpc
from reachy_sdk_api import sensor_pb2, sensor_pb2_grpc
from reachy_sdk_api import kinematics_pb2
from reachy_sdk_api import arm_kinematics_pb2, arm_kinematics_pb2_grpc
from reachy_sdk_api import fullbody_cartesian_command_pb2, fullbody_cartesian_command_pb2_grpc
from reachy_sdk_api import fan_pb2, fan_pb2_grpc
from reachy_sdk_api import mobile_platform_reachy_pb2, mobile_platform_reachy_pb2_grpc

from .utils import jointstate_pb_from_request

proto_arm_side_to_str = {
    arm_kinematics_pb2.ArmSide.LEFT: 'left',
    arm_kinematics_pb2.ArmSide.RIGHT: 'right',
}


class ReachySDKServer(Node,
                      joint_pb2_grpc.JointServiceServicer,
                      sensor_pb2_grpc.SensorServiceServicer,
                      arm_kinematics_pb2_grpc.ArmKinematicsServicer,
                      fullbody_cartesian_command_pb2_grpc.FullBodyCartesianCommandServiceServicer,
                      fan_pb2_grpc.FanControllerServiceServicer,
                      mobile_platform_reachy_pb2_grpc.MobileBasePresenceServiceServicer,
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
        self.fans: Dict[str, bool] = OrderedDict()
        self.setup()

        self.id2names = {i: name for i, name in enumerate(self.joints.keys())}
        self.names2ids = {name: i for i, name in enumerate(self.joints.keys())}
        for name, uid in self.names2ids.items():
            self.joints[name]['uid'] = uid

        self.logger.info('Launching pub/sub/srv...')

        self.compliant_client = self.create_client(SetJointCompliancy, 'set_joint_compliancy')
        while not self.compliant_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.compliant_client.srv_name} not available, waiting again...')

        self.set_pid_client = self.create_client(SetJointPidGains, 'set_joint_pid')
        while not self.set_pid_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.set_pid_client.srv_name} not available, waiting again...')

        self.set_fan_client = self.create_client(SetFanState, 'set_fan_state')
        while not self.set_fan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.set_fan_client.srv_name} not available, waiting again...')

        self.get_reachy_model_client = self.create_client(GetReachyModel, 'get_reachy_model')
        while not self.get_reachy_model_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.get_reachy_model_client.srv_name} not available, waiting again...')

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

        self.fan_states_sub = self.create_subscription(
            msg_type=FanState, topic='fan_states',
            callback=self.on_fan_states, qos_profile=5,
        )

        self.joint_goals_pub = self.create_publisher(
            msg_type=JointState, topic='joint_goals', qos_profile=5,
        )

        self.grippers_goals_publisher = self.create_publisher(
            msg_type=GripperMsg, topic='grippers', qos_profile=5,
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

        for cli in [
            self.left_arm_fk, self.left_arm_ik,
            self.right_arm_fk, self.right_arm_ik,
        ]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'service {cli.srv_name} not available, waiting again...')

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

        while True:
            fut = joint_fullstate_client.call_async(GetJointFullState.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=1)
            if not fut.done():
                self.get_logger().info(f'service {joint_fullstate_client.srv_name} timeout, trying again...')
                continue
            break
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

    def on_fan_states(self, fan_state: FanState) -> None:
        """Update fan state (on or off) on fan_state msg."""
        if not fan_state.name:
            return
        for name, state in zip(fan_state.name, fan_state.on):
            self.fans[name] = state

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
            joint_goals.name = [j for j in self.joints.keys() if not j.endswith('gripper')]

            if self.should_publish_position.is_set():
                joint_goals.position = [j['goal_position'] for j in self.joints.values() if not j['name'].endswith('gripper')]
                self.should_publish_position.clear()

            if self.should_publish_velocity.is_set():
                joint_goals.velocity = [j['speed_limit'] for j in self.joints.values() if not j['name'].endswith('gripper')]
                self.should_publish_velocity.clear()

            if self.should_publish_effort.is_set():
                joint_goals.effort = [j['torque_limit'] for j in self.joints.values() if not j['name'].endswith('gripper')]
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
                name = self._joint_id_to_name(cmd.id)

                names.append(name)
                values.append(cmd.compliant.value)

                if not cmd.compliant.value and self.joints[name]['compliant']:
                    # If turning stiff we reset any obsolete goal_position we may have
                    self.joints[name]['goal_position'] = self.joints[name]['present_position']

        if names:
            request = SetJointCompliancy.Request()
            request.name = names
            request.compliancy = values

            # TODO: Should be re-written using asyncio
            future = self.compliant_client.call_async(request)
            for _ in range(1000):
                if future.done():
                    success = future.result().success
                    for name, val in zip(names, values):
                        self.joints[name]['compliant'] = val
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
                    for name, val in zip(names_pid, pid_gains):
                        self.joints[name]['pid'] = self._repr_ros_pid(val)
                    break
                time.sleep(0.001)
            else:
                success = False

        use_goal_pos, use_goal_vel, use_goal_eff = False, False, False
        grippers_commands = []
        for cmd in commands:
            name = self._joint_id_to_name(cmd.id)

            if name.endswith('gripper'):
                if cmd.HasField('goal_position'):
                    grippers_commands.append(cmd)

            else:
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
        if grippers_commands:
            self.handle_grippers_command(grippers_commands)
        return success

    def handle_grippers_command(self, commands: List[joint_pb2.JointCommand]):
        msg = GripperMsg()
        msg.name = [self._joint_id_to_name(cmd.id) for cmd in commands]
        msg.goal_position = [float(cmd.goal_position.value) for cmd in commands]

        if msg.name:
            self.grippers_goals_publisher.publish(msg)

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
        if list(self.force_sensors.keys()) != []:
            uids, names = zip(*enumerate(self.force_sensors.keys()))
        else:
            uids, names = [], []
        return sensor_pb2.SensorsId(names=names, uids=uids)

    def GetSensorsState(self, request: sensor_pb2.SensorsStateRequest, context) -> sensor_pb2.SensorsState:
        """Get the requested sensors value."""
        forces = list(self.force_sensors.values())

        params = {}
        params['ids'] = request.ids
        params['states'] = [
            sensor_pb2.SensorState(force_sensor_state=sensor_pb2.ForceSensorState(force=forces[id.uid]))
            for id in request.ids
        ]

        return sensor_pb2.SensorsState(**params)

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
            success=resp.success,
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
                q = request.neck.q

                joints = [f'neck_{axis}' for axis in ('roll', 'pitch', 'yaw')]
                rpy = Rotation.from_quat((q.x, q.y, q.z, q.w)).as_euler('XYZ')

                goal_position.update({
                    joint: val
                    for joint, val in zip(joints, rpy)
                })

            for name, pos in goal_position.items():
                try:
                    self.joints[name]['goal_position'] = pos
                except KeyError:
                    self.logger.warning(f'Could not set goal position to unknown joint "{name}"')
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

    # Fan state handler
    def _fan_ids_request_to_str(self, ids_request: List[fan_pb2.FanId]) -> List[str]:
        fan_names = list(self.fans.keys())
        fans_requested = []

        for fid in ids_request:
            if fid.HasField('uid'):
                fans_requested.append(fan_names[fid.uid])
            else:
                fans_requested.append(fid.name)
        return fans_requested

    def GetAllFansId(self, request: Empty, context) -> fan_pb2.FansId:
        """Get the id of each fan in Reachy."""
        if self.fans == OrderedDict():
            return fan_pb2.FansId(names=[], uids=[])

        uids, names = zip(*enumerate(self.fans.keys()))
        return fan_pb2.FansId(names=names, uids=uids)

    def GetFansState(self, request: fan_pb2.FansStateRequest, context) -> fan_pb2.FansState:
        """Get the state of the requested fans."""
        params = {}
        params['ids'] = request.ids
        params['states'] = [fan_pb2.FanState(on=self.fans[f]) for f in self._fan_ids_request_to_str(request.ids)]
        return fan_pb2.FansState(**params)

    def SendFansCommands(self, request: fan_pb2.FansCommand, context) -> fan_pb2.FansCommandAck:
        """Set the states of the requested fans."""
        ros_request = SetFanState.Request(
            name=self._fan_ids_request_to_str([fc.id for fc in request.commands]),
            state=[fc.on for fc in request.commands],
        )
        future = self.set_fan_client.call_async(ros_request)
        # TODO: Should be re-written using asyncio
        for _ in range(1000):
            if future.done():
                success = future.result().success
                break
            time.sleep(0.001)
        return fan_pb2.FansCommandAck(success=success)

    # Mobile base presence handler
    def GetMobileBasePresence(
                            self,
                            request: Empty,
                            context) -> mobile_platform_reachy_pb2.MobileBasePresence:
        """Return if a mobile base is in Reachy's config file.

        If yes, return the mobile base version.
        """
        presence = False
        version = '0.0'

        model = check_output(['reachy-identify-zuuu-model']).strip().decode()

        if model and model != 'None':
            presence = True
            version = float(model)

        response = mobile_platform_reachy_pb2.MobileBasePresence(
            presence=BoolValue(value=presence),
            model_version=FloatValue(value=version),
        )
        return response


def main():
    """Run the Node and the gRPC server."""
    rclpy.init()

    sdk_server = ReachySDKServer(node_name='reachy_sdk_server')

    server = grpc.server(thread_pool=ThreadPoolExecutor(max_workers=10))
    joint_pb2_grpc.add_JointServiceServicer_to_server(sdk_server, server)
    sensor_pb2_grpc.add_SensorServiceServicer_to_server(sdk_server, server)
    arm_kinematics_pb2_grpc.add_ArmKinematicsServicer_to_server(sdk_server, server)
    fullbody_cartesian_command_pb2_grpc.add_FullBodyCartesianCommandServiceServicer_to_server(sdk_server, server)
    fan_pb2_grpc.add_FanControllerServiceServicer_to_server(sdk_server, server)
    mobile_platform_reachy_pb2_grpc.add_MobileBasePresenceServiceServicer_to_server(sdk_server, server)

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
