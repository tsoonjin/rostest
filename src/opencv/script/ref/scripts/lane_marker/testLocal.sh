#!/bin/bash

if [[ $1 == "acoustic" ]]; then
    rosrun vision run.py lane_marker.acousticStates _alone:=True _image:=/bot_camera/camera/image_raw_thien
else
    rosrun vision run.py lane_marker.states _alone:=True _image:=/bot_camera/camera/image_raw_thien
fi
