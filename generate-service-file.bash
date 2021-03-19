tee reachy_sdk_server.service <<EOF
[Unit]
Description=Reachy SDK server service
Wants=network-online.target
After=network.target network-online.target
[Service]
PIDFile=/var/run/reachy_sdk_server.pid
ExecStart=/usr/bin/bash $PWD/launch_all.bash
Environment="PATH=$PATH:$(dirname $(which reachy-identify-model))"
User=$(whoami)
Group=$(whoami)
Type=simple
[Install]
WantedBy=multi-user.target
EOF

