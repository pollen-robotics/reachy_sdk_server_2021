"""GRPC server to restart/stop reachy_sdk_server.service."""
import grpc
from concurrent.futures import ThreadPoolExecutor

from reachy_sdk_api import restart_signal_pb2, restart_signal_pb2_grpc
from tools import send_service_signal


class RestartServer(restart_signal_pb2_grpc.RestartServiceServicer):
    """Restart server class."""

    def __init__(self) -> None:
        """Init grpc server."""
        super().__init__()

    def SendRestartSignal(self, request: restart_signal_pb2.RestartCmd, context) -> restart_signal_pb2.RestartSignalAck:
        """Restart or stop reachy_sdk_server.service."""
        restart_grpc_cmd_to_str = {
            restart_signal_pb2.SignalType.RESTART: 'restart',
            restart_signal_pb2.SignalType.STOP: 'stop',
        }
        send_service_signal(restart_grpc_cmd_to_str[request.cmd])
        return restart_signal_pb2.RestartSignalAck(success=True)


def main():
    """Run grpc server."""
    test_server = RestartServer()
    server = grpc.server(thread_pool=ThreadPoolExecutor(max_workers=10))
    restart_signal_pb2_grpc.add_RestartServiceServicer_to_server(test_server, server)

    server.add_insecure_port('[::]:50059')
    server.start()
    server.wait_for_termination()


if __name__ == '__main__':
    main()
