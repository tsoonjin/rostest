#!/usr/bin/env python

import rospy 
from custom_msgs.srv import *

class Client:
    def __init__(self):
        rospy.init_node('client')
        rospy.wait_for_service('navigation', timeout=5)
        self.res = rospy.ServiceProxy('navigation', euler)
        data = self.res(pos_x=4.4, pos_y=3.3)
        rospy.loginfo(data.pos_z)
        rospy.spin()

if __name__ == '__main__':
    Client()

