"""Utility protobuf factories."""

from typing import Dict, List

from google.protobuf.wrappers_pb2 import BoolValue, FloatValue, UInt32Value

from reachy_sdk_api.joint_pb2 import JointField, JointState, PIDGains, PIDValue, ComplianceMarginSlope


def parse_fields(fields: List) -> List[int]:
    """Parse JointField (handles NONE, ALL specific cases)."""
    if JointField.NONE in fields:
        return []

    if JointField.ALL in fields:
        fields = JointField.values()
        fields.remove(JointField.ALL)
        fields.remove(JointField.NONE)

    return fields


def jointstate_pb_from_request(joint: Dict, fields: List) -> JointState:
    """Create a protobuf JointState msg from joint data and specified fields."""
    fields = parse_fields(fields)

    params = {}

    for field in fields:
        if field == JointField.NAME:
            params['name'] = joint['name']

        elif field == JointField.UID:
            _inject_pb_value(params, joint, 'uid', UInt32Value)

        elif field == JointField.PRESENT_POSITION and joint['present_position'] is not None:
            _inject_pb_value(params, joint, 'present_position', FloatValue)

        elif field == JointField.PRESENT_SPEED and joint['present_speed'] is not None:
            _inject_pb_value(params, joint, 'present_speed', FloatValue)

        elif field == JointField.PRESENT_LOAD and joint['present_load'] is not None:
            _inject_pb_value(params, joint, 'present_load', FloatValue)

        elif field == JointField.TEMPERATURE:
            _inject_pb_value(params, joint, 'temperature', FloatValue)

        elif field == JointField.COMPLIANT:
            _inject_pb_value(params, joint, 'compliant', BoolValue)

        elif field == JointField.GOAL_POSITION:
            _inject_pb_value(params, joint, 'goal_position', FloatValue)

        elif field == JointField.SPEED_LIMIT:
            _inject_pb_value(params, joint, 'speed_limit', FloatValue)

        elif field == JointField.TORQUE_LIMIT:
            _inject_pb_value(params, joint, 'torque_limit', FloatValue)

        elif field == JointField.PID:
            _inject_pid(params, joint)

    return JointState(**params)


def _inject_pb_value(params, joint, field, value_type):
    value = value_type()
    value.value = joint[field]
    params[field] = value


def _inject_pid(params, joint):
    if len(joint['pid']) == 3:
        params['pid'] = PIDValue(
            pid=PIDGains(
                p=joint['pid'][0],
                i=joint['pid'][1],
                d=joint['pid'][2],
            )
            )
    elif len(joint['pid']) == 4:
        params['pid'] = PIDValue(
            compliance=ComplianceMarginSlope(
                cw_compliance_margin=joint['pid'][0],
                ccw_compliance_margin=joint['pid'][1],
                cw_compliance_slope=joint['pid'][2],
                ccw_compliance_slope=joint['pid'][3],
            )
        )
