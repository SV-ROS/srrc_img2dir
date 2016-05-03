#!/usr/bin/env bash

source /home/ubuntu/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=192.168.0.60
export ROS_MASTER_URI=http://192.168.0.51:11311/
exec "$@"


