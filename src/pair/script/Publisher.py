#!/usr/bin/env python
import rospy
from custom_msgs.msg import Pos

class Publisher:
    
    def __init__(self):
        print "Publisher created"
        rospy.init_node('publisher')
        self.pub = rospy.Publisher('Position', Pos, queue_size=1)
        rate = rospy.Rate(10);
        while not rospy.is_shutdown():
            self.pub.publish(pos_x=1.1, pos_y=2.2)
            rate.sleep()

if __name__ == "__main__":
    try:
        publisher = Publisher();
    except rospy.ROSInterruptException:
        pass
