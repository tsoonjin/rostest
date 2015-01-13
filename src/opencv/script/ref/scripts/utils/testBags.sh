#!/bin/bash

roscore &
roslaunch launch uncompressbags.launch &
rosrun controls PID_Controller &
rqt &
