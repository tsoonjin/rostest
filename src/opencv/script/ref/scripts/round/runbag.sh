#!/bin/bash

roscore & rosrun controls PID_Controller & sh ./muslimRepub.sh & rqt

