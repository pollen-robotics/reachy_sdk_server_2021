"""..."""
from subprocess import call


def send_service_signal(signal_type: str):
    """..."""
    if signal_type not in ['restart', 'stop']:
        raise ValueError("Signal sent to reachy_sdk_server.service should be either 'restart' or 'stop'.")

    call(['sudo', 'systemctl', signal_type, 'reachy_sdk_server.service'])

    call(['systemctl', '--user', 'stop', 'webrtc_server.service'])
    call(['sudo', 'systemctl', signal_type, 'signaling_server.service'])

    if signal_type == 'restart':
        call(['systemctl', '--user', 'start', 'webrtc_server.service'])
