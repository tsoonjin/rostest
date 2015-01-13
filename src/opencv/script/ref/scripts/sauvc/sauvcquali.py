#!/usr/bin/env python

#Move forward 10m

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

import roslib; roslib.load_manifest('vision')
import rospy
import actionlib

import signal 

class Qualifier:
    def __init__(self):
        self.isAborted = False
        self.isKilled = False
        self.depth_setpoint = 0.2
        self.yaw = 0.0
        self.testing = rospy.get_param("~testing", False)
        self.yaw_sub = rospy.Subscriber('/euler', compass_data, self.yaw_callback)
        self.locomotionClient = actionlib.SimpleActionClient("LocomotionServer", bbauv_msgs.msg.ControllerAction)
        
        
        signal.signal(signal.SIGINT, self.userQuit)
        
        if self.isAborted or self.isKilled:
            stopRobot()
        
        rospy.loginfo("Let's qualify!")
        
        #Locomotion server
        try:
            self.locomotionClient.wait_for_server(timeout=rospy.Duration(5))
        except:
            rospy.logerr("Locomotion server timeout!")
            self.isKilled = True
            
        self.sendMovement(forward=10.0)   
        self.stopRobot()        
        
    def yaw_callback(self, msg):
        self.yaw = msg.yaw
        
    def userQuit(self, signal, frame):
        self.isAborted = True
        self.isKilled = True
        
    def stopRobot(self):
        self.sendMovement(forward=0.0, sidemove=0.0)
        
    def sendMovement(self, forward=0.0, heading=None, sidemove=0.0, depth=None):
        depth = depth if depth else self.depth_setpoint
        heading = heading if heading else self.yaw
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=forward, heading_setpoint=heading,
                                             sidemove_setpoint=sidemove, depth_setpoint=depth)
        self.locomotionClient.send_goal(goal)
        self.locomotionClient.wait_for_result(rospy.Duration(0.3))

if __name__ == "__main__":
    rospy.init_node("SAUVC_Qualifier")
    qualifier = Qualifier()
    rospy.spin()
        
        
        
        