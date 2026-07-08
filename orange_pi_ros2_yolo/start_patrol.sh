#!/bin/bash

cd ~/ros2_car_ws

source /opt/ros/humble/setup.bash
source ~/ros2_car_ws/install/setup.bash

sudo fuser -k /dev/ttyACM0 2>/dev/null
sudo chmod 666 /dev/ttyACM0

echo "[start] warehouse patrol state machine"
ros2 run car_serial_control yolo_auto
