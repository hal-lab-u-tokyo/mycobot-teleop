#!/bin/bash

RELATIVE_PATH="$(dirname "$0")"
cd "$RELATIVE_PATH" || exit 1
SERVER_SCRIPT_PATH="./mycobot_server_fixed.py"
MYCOBOT_IP="192.168.50.2"
MYCOBOT_PORT="22"
MYCOBOT_USER="pi"

ssh $MYCOBOT_USER@$MYCOBOT_IP -p $MYCOBOT_PORT "mkdir -p /home/pi"
scp $SERVER_SCRIPT_PATH $MYCOBOT_USER@$MYCOBOT_IP:/home/pi/mycobot_server.py 