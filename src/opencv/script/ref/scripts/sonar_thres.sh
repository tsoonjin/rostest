#!/bin/bash

ROS_NAMESPACE=/front_camera/camera rosrun image_transport republish compressed in:=image_raw raw out:=image_raw_lynnette &
rosrun image_transport republish compressed in:=sonar_image raw out:=sonar_image_lynnette &

if [[ $1 == "True" ]];
then
echo $1
rosrun vision sonarthres.py _image:=/front_camera/camera/image_raw_lynnette _sonar:=/sonar_image_lynnette _testing:=True
else
echo $1
rosrun vision sonarthres.py _image:=/front_camera/camera/image_raw_lynnette _sonar:=/sonar_image_lynnette _testing:=False
fi
