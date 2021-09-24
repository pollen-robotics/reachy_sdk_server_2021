tee reachy_sdk_restart_server.service <<EOF
[Unit]
Description=Reachy SDK restart server service
Wants=network-online.target
After=network.target network-online.target
[Service]
PIDFile=/var/run/reachy_sdk_restart_server.pid
ExecStart=/usr/bin/python3.8 $PWD/reachy_sdk_server/restart_server.py
User=$(whoami)
Group=$(whoami)
Type=simple
[Install]
WantedBy=multi-user.target
EOF
