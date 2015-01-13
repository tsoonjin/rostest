#!/usr/bin/env python

import rospy
from custom_msgs.msg import *

class Subscriber:
    def __init__(self):
        rospy.init_node('subscriber');
        self.subscriber = rospy.Subscriber('Position', Pos, self.pos_callback)
        rospy.spin();

    def pos_callback(self, data):
        rospy.loginfo('X: ' + str(data.pos_x) + ' Y: ' + str(data.pos_y));

if __name__ == '__main__':
    try:
        sub = Subscriber();
    except rospy.ROSInterruptException:
        pass
