import time
from subprocess import run, PIPE
from concurrent.futures import ThreadPoolExecutor
from queue import Empty
import grpc
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue, UInt32Value

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

from reachy_sdk_api import mobile_platform_reachy_pb2, mobile_platform_reachy_pb2_grpc

from zuuu_interfaces.srv import SetZuuuMode, GetOdometry, ResetOdometry
from zuuu_interfaces.srv import GoToXYTheta, IsGoToFinished, DistanceToGoal
from zuuu_interfaces.srv import SetSpeed, GetBatteryVoltage


class MobileBaseServer(
                        Node,
                        mobile_platform_reachy_pb2_grpc.MobileBasePresenceServiceServicer,
                        mobile_platform_reachy_pb2_grpc.MobilityServiceServicer,
                        ):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()

        self.clock = self.get_clock()

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.set_speed_client = self.create_client(SetSpeed, 'SetSpeed')
        while not self.set_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service SetSpeed not available, waiting again...')

        self.go_to_client = self.create_client(GoToXYTheta, 'GoToXYTheta')
        while not self.go_to_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service GoToXYTheta not available, waiting again...')

        self.distance_to_goal_client = self.create_client(DistanceToGoal, 'DistanceToGoal')
        while not self.distance_to_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service DistanceToGoal not available, waiting again...')

        self.set_zuuu_mode_client = self.create_client(SetZuuuMode, 'SetZuuuMode')
        while not self.set_zuuu_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service SetZuuuMode not available, waiting again...')

        self.get_battery_voltage_client = self.create_client(GetBatteryVoltage, 'GetBatteryVoltage')
        while not self.get_battery_voltage_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service GetBatteryVoltage not available, waiting again...')

        self.get_odometry_client = self.create_client(GetOdometry, 'GetOdometry')
        while not self.get_odometry_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service GetOdometry not available, waiting again...')

        self.reset_odometry_client = self.create_client(ResetOdometry, 'ResetOdometry')
        while not self.reset_odometry_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service ResetOdometry not available, waiting again...')

        self.logger.info('Initialized mobile base server.')

    def SendDirection(
                    self,
                    request: mobile_platform_reachy_pb2.TargetDirectionCommand,
                    context) -> mobile_platform_reachy_pb2.TargetDirectionCommandAck:
        """Send a speed command for the mobile base expressed in SI units """
        self.logger.info('Received send direction order.')
        twist = Twist()
        twist.linear.x = request.direction.x.value
        twist.linear.y = request.direction.y.value
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = request.direction.theta.value
        self.cmd_vel_pub.publish(twist)

        return mobile_platform_reachy_pb2.TargetDirectionCommandAck(success=BoolValue(value=True))

    def SendSetSpeed(
                    self,
                    request: mobile_platform_reachy_pb2.SetSpeedVector,
                    context) -> mobile_platform_reachy_pb2.SetSpeedAck:
        req = SetSpeed.Request()
        req.duration = request.duration.value
        req.x_vel = request.x_vel.value
        req.y_vel = request.y_vel.value
        req.rot_vel = request.rot_vel.value

        self.set_speed_client.call_async(req)
        return mobile_platform_reachy_pb2.SetSpeedAck(success=BoolValue(value=True))

    def SendGoTo(
                self,
                request: mobile_platform_reachy_pb2.GoToVector,
                context) -> mobile_platform_reachy_pb2.GoToAck:
        req = GoToXYTheta.Request()
        req.x_goal = request.x_goal.value
        req.y_goal = request.y_goal.value
        req.theta_goal = request.theta_goal.value

        self.go_to_client.call_async(req)
        return mobile_platform_reachy_pb2.GoToAck(success=BoolValue(value=True))

    def DistanceToGoal(self, request, context):
        response = mobile_platform_reachy_pb2.DistanceToGoalVector(
            delta_x=FloatValue(value=0.0),
            delta_y=FloatValue(value=0.0),
            delta_theta=FloatValue(value=0.0),
            distance=FloatValue(value=0.0))

        req = DistanceToGoal.Request()

        future = self.distance_to_goal_client.call_async(req)
        for _ in range(1000):
            if future.done():
                ros_response = future.result()
                print(ros_response)
                response.delta_x.value = ros_response.delta_x
                response.delta_y.value = ros_response.delta_y
                response.delta_theta.value = ros_response.delta_theta
                response.distance.value = ros_response.distance
                break
            time.sleep(0.001)
        return response

    def SetControlMode(
                    self,
                    request: mobile_platform_reachy_pb2.ControlModeCommand,
                    context) -> mobile_platform_reachy_pb2.ControlModeCommandAck:

        mode = mobile_platform_reachy_pb2.ControlModePossiblities.keys()[request.mode]

        if mode == 'NONE_CONTROL_MODE':
            return mobile_platform_reachy_pb2.ControlModeCommandAck(success=BoolValue(value=False))

        run(f'ros2 param set /zuuu_hal control_mode {mode}', stdout=PIPE, shell=True)
        return mobile_platform_reachy_pb2.ControlModeCommandAck(success=BoolValue(value=True))

    def SetZuuuMode(
                self,
                request: mobile_platform_reachy_pb2.ZuuuModeCommand,
                context) -> mobile_platform_reachy_pb2.ZuuuModeCommandAck:
        mode = mobile_platform_reachy_pb2.ZuuuModePossiblities.keys()[request.mode]

        if mode == 'NONE_ZUUU_MODE':
            return mobile_platform_reachy_pb2.ZuuuModeCommandAck(success=BoolValue(value=False))

        req = SetZuuuMode.Request()
        req.mode = mode
        self.set_zuuu_mode_client.call_async(req)
        return mobile_platform_reachy_pb2.ZuuuModeCommandAck(success=BoolValue(value=True))

    def GetBatteryLevel(
                self,
                request: Empty,
                context) -> mobile_platform_reachy_pb2.BatteryLevel:
        req = GetBatteryVoltage.Request()

        response = mobile_platform_reachy_pb2.BatteryLevel(
            level=FloatValue(value=0.0)
            )

        future = self.get_battery_voltage_client.call_async(req)
        for _ in range(1000):
            if future.done():
                ros_response = future.result()
                response.level.value = ros_response.voltage
                break
            time.sleep(0.001)
        return response

    def GetOdometry(
                self,
                request: Empty,
                context) -> mobile_platform_reachy_pb2.OdometryVector:
        req = GetOdometry.Request()
        response = mobile_platform_reachy_pb2.OdometryVector(
            x=FloatValue(value=0.0),
            y=FloatValue(value=0.0),
            theta=FloatValue(value=0.0),
        )
        future = self.get_odometry_client.call_async(req)
        for _ in range(1000):
            if future.done():
                ros_response = future.result()
                print(ros_response)
                response.x.value = ros_response.x
                response.y.value = ros_response.y
                response.theta.value = ros_response.theta
                break
            time.sleep(0.001)
        return response

    def ResetOdometry(
                self,
                request: Empty,
                context) -> mobile_platform_reachy_pb2.ResetOdometryAck:
        req = ResetOdometry.Request()
        self.reset_odometry_client.call_async(req)
        return mobile_platform_reachy_pb2.ResetOdometryAck(success=BoolValue(value=True))


def main():
    """Run the Node and the gRPC server."""
    rclpy.init()

    mobile_base_server = MobileBaseServer(node_name='mobile_base_server')

    options = [
         ('grpc.max_send_message_length', 250000),
         ('grpc.max_receive_message_length', 250000),
         ]

    server = grpc.server(thread_pool=ThreadPoolExecutor(max_workers=10), options=options)
    mobile_platform_reachy_pb2_grpc.add_MobilityServiceServicer_to_server(mobile_base_server, server)
    mobile_platform_reachy_pb2_grpc.add_MobileBasePresenceServiceServicer_to_server(mobile_base_server, server)

    server.add_insecure_port('[::]:50061')
    server.start()

    try:
        rclpy.spin(mobile_base_server)
    except KeyboardInterrupt:
        pass

    server.stop(grace=None)
    server.wait_for_termination()

    mobile_base_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()