tee reachy_sdk_server.service <<EOF
[Unit]
Description=Reachy SDK server service

[Service]
ExecStart=/usr/bin/bash $PWD/launch_all.bash
Environment="PATH=$PATH:$(dirname $(which reachy-identify-model))"

[Install]
WantedBy=default.target
EOF

