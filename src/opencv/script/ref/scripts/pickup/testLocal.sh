#!/bin/bash

if [[ $1 == "drop" ]]; then
    rosrun vision run.py pickup.dropStates _alone:=True _image:=bot_camera/camera/image_raw_thien
else
    rosrun vision run.py pickup.states _alone:=True _image:=bot_camera/camera/image_raw_thien
fi
