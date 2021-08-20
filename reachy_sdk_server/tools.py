"""..."""
from subprocess import call


def send_service_signal(signal_type: str):
    """..."""
    if signal_type not in ['restart', 'stop']:
        raise ValueError("Signal sent to reachy_sdk_server.service should be either 'restart' or 'stop'.")
    call(['sudo', 'systemctl', signal_type, 'reachy_sdk_server.service'])
