#!/bin/bash


cd /home/rover/webrtc-camera-server; sudo docker compose kill

sudo -i -u rover bash << EOF

source "/home/rover/.bashrc"

tmux kill-session -t ros

EOF
