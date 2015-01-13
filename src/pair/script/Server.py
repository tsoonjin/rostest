#!/usr/bin/env python 

import rospy
from custom_msgs.srv import *

class Server:
    def __init__(self):
        rospy.init_node('server')
        self.server = rospy.Service('navigation', euler, self.handler)
        rospy.spin()

    def handler(self, req):
        rospy.loginfo('X: ' + str(req.pos_x) + ' Y: ' + str(req.pos_y))
        return eulerResponse(pos_z = 3)

if __name__ == '__main__':
    Server()
