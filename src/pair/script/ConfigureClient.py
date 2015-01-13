#!/usr/bin/env python

import rospy
import random

PACKAGE = "pair"

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Red Threshold: " + config['red_threshold'] + " Name: " + config['input_name'] + " Is Finish: " + config['is_finish'])


def main():
    rospy.init_node("configure")
    client = dynamic_reconfigure.client.Client(PACKAGE, timeout=30, config_callback=callback)
    while not rospy.is_shutdown():
        x = random.randrange(30,100) 
        client.update_configuration({'red_threshold':x, 'input_name':"d", 'is_finish':True})

if __name__ =="__main__":
    main()
