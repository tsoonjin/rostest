#!/bin/bash

cd ~/Code/bbauv/src/gui/controlpanel/src

rosrun gui vision.py _image:=/front_camera/camera/image_color_alex _filter:=/Vision/image_filter_opt_alex & 

