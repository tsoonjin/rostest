#!/usr/bin/env python 

import rospy
from dynamic_reconfigure.server import Server
from pair.cfg import dynamicConfig

def callback(config,level):
    rospy.loginfo("Red_Threshold: " + repr(config['red_threshold']))
    return config

if __name__ == '__main__':
    rospy.init_node("server")
    srv = Server(dynamicConfig, callback)
    rospy.spin()
