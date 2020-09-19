#! /bin/bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

source ~/venv/bin/activate
python -m http.server

#roslaunch rosbridge_server rosbridge_websocket.launch