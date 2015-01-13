#! /usr/bin/env python

import roslib; roslib.load_manifest('vision')
import rospy


if __name__ == '__main__':
    rospy.init_node('pytalker')
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
