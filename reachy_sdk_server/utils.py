"""Utility protobuf factories."""

from typing import Dict, List

from google.protobuf.timestamp_pb2 import Timestamp
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue

from reachy_sdk_api.joint_state_pb2 import JointState, JointStateField


def parse_fields(fields: List) -> List:
    """Parse JointStateField (handles NONE, ALL specific cases)."""
    if JointStateField.NONE in fields:
        return []

    if JointStateField.ALL in fields:
        fields = JointStateField.values()
        fields.remove(JointStateField.ALL)
        fields.remove(JointStateField.NONE)

    return fields


def jointstate_pb_from_request(joint: Dict, fields: List, timestamp: bool) -> JointState:
    """Create a protobuf JointState msg from joint data and specified fields."""
    fields = parse_fields(fields)

    params = {}

    for field in fields:
        if field == JointStateField.NAME:
            params['name'] = joint['name']

        elif field == JointStateField.PRESENT_POSITION and joint['present_position'] is not None:
            _inject_pb_value(params, joint, 'present_position', FloatValue)

        elif field == JointStateField.PRESENT_SPEED and joint['present_speed'] is not None:
            _inject_pb_value(params, joint, 'present_speed', FloatValue)

        elif field == JointStateField.PRESENT_LOAD and joint['present_load'] is not None:
            _inject_pb_value(params, joint, 'present_load', FloatValue)

        elif field == JointStateField.TEMPERATURE:
            _inject_pb_value(params, joint, 'temperature', FloatValue)

        elif field == JointStateField.COMPLIANT:
            _inject_pb_value(params, joint, 'compliant', BoolValue)

        elif field == JointStateField.GOAL_POSITION:
            _inject_pb_value(params, joint, 'goal_position', FloatValue)

        elif field == JointStateField.SPEED_LIMIT:
            _inject_pb_value(params, joint, 'speed_limit', FloatValue)

        elif field == JointStateField.TORQUE_LIMIT:
            _inject_pb_value(params, joint, 'torque_limit', FloatValue)

        # elif field == JointStateField.PID:
        #     params['pid'] = random_pid()

    if timestamp:
        params['timestamp'] = Timestamp()
        params['timestamp'].GetCurrentTime()

    return JointState(**params)


def _inject_pb_value(params, joint, field, value_type):
    value = value_type()
    value.value = joint[field]
    params[field] = value
