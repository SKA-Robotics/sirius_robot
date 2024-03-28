#!/bin/bash

slcand -o -s3 -t hw -S 115200 /dev/ttyACM0
ip link set up slcan0

modprobe -r uvcvideo
modprobe uvcvideo
echo 1 | sudo tee /sys/module/uvcvideo/parameters/bandwidth_cap

sleep 5

#modprobe v4l2loopback video_nr=69

sudo -i -u rover bash << EOF

source "/home/rover/.bashrc"

TERM=xterm-256color tmux new-session -d -s ros -n nodes "bash -l -i -c \"source /home/rover/software/ros1/devel/setup.bash; sleep 5; mon launch sirius_bringup sirius.launch\"; bash -i"
TERM=xterm-256color tmux new-window -d -t ros: -n master "source /home/rover/software/ros1/devel/setup.bash; roscore; bash -i"
TERM=xterm-256color tmux new-window -d -t ros: -n web_server "source /home/rover/.nvm/nvm.sh; cd /home/rover/robot-web-interface/server; npm run start; bash -i"
TERM=xterm-256color tmux new-window -d -t ros: -n cameras "sleep 5; cd /home/rover/webrtc-camera-server; sudo docker compose run gstreamer-cameras; bash -i"

EOF
