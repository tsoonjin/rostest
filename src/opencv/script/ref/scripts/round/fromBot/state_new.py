#!/usr/bin/env python 
import roslib 
import rospy 
import actionlib

import smach
import smach_ros

import sys 
import os 
import numpy as np
import time

from sensor_msgs.msg import Image
from bbauv_msgs.msg import * 
from bbauv_msgs.srv import * 
from utils.utils import Utils

from vision import PoleVision
from comms import Comms
from config import Config

#GLOBAL
con = Config()
class Disengage(smach.State):
    
    def __init__(self, pipe):
        smach.State.__init__(self, outcomes=['init', 'aborted', 'completed', 'test'])
        self.pipe = pipe

    def execute(self, userdata):
        rospy.loginfo("In DISENGAGE")

        while not self.pipe.isStart:
            rospy.loginfo("Ready to serve, my lord")
            rospy.sleep(rospy.Duration(2.0))

        if self.pipe.isTest:
            self.pipe.registerCurr()
            return 'test'

        elif self.pipe.isAborted:
            self.pipe.send2Mission(task_complete_request=False, fail_request=True)
            rospy.loginfo("Aborted")
            return 'aborted'

        elif self.pipe.isDone:
            self.pipe.send2Mission(task_complete_request=True, fail_request=False)
            rospy.loginfo("Completed")
            return 'completed'

        else:
            self.pipe.registerCurr()
            self.pipe.initTime = time.time()
            rospy.loginfo("Initialised")
            return "init"


class Test(smach.State):
    def __init__(self, pipe):
        smach.State.__init__(self, outcomes=['done'])
        rospy.loginfo("Ready to do stationary test")
        self.con = Config()
        self.pipe = pipe

        
    def execute(self, userdata):
        while True:
            pass


class Search(smach.State):
    def __init__(self, pipe):
        smach.State.__init__(self, outcomes=['lost', 'detected', 'aborted'])
        self.pipe = pipe
        self.con = Config()
        self.found = None

    def execute(self, userdata):

        self.found = 0
        self.start_time = time.time()
        time.sleep(5)

        while self.found < 5 and not self.pipe.isAborted:
            rospy.loginfo("Found: " + str(self.found))
            
            if time.time() - self.start_time > 30.0:
                rospy.loginfo("Timeout, just wack la")
                self.pipe.sendMovement(sidemove=0.6)
                self.pipe.sendMovement(forward=2.5)
                self.pipe.isAborted = True
                return 'lost'

            elif self.pipe.red_img.detected and self.pipe.green_img.detected:

                self.found += 1

                if not self.pipe.green_img.ends:
                    rospy.loginfo("Alligning with green poles")
                    green_center = self.pipe.green_img.centroid[0]
                    move_x = (self.con.center_w - green_center)*self.con.mult_x/(self.con.screen_w)
                    self.pipe.sendMovement(sidemove=move_x, wait=False)

            elif self.pipe.red_img.detected:
                
                move_x = self.pipe.red_img.deltaX*self.con.mult_x
                self.pipe.sendMovement(sidemove=move_x, wait=False)

        self.found = 0
        if self.pipe.isAborted:
            return 'aborted'
        else:
            rospy.loginfo("Pole detected: Commencing dance move")
            return 'detected'
            
                    

class Position(smach.State):
    def __init__(self, pipe):
        smach.State.__init__(self, outcomes=['locked', 'check', 'find', 'aborted'])
        self.pipe = pipe
        self.con = Config()

    def execute(self, userdata):
        area = 0
        forward_mov = 0
        if self.pipe.red_img.detected and self.pipe.green_img.detected:
            if abs(self.pipe.red_img.deltaX) < 0.05:
                while self.pipe.red_img.area < 10000:     
                    self.pipe.sendMovement(forward=2.0)
                self.pipe.setDepth = self.pipe.currDepth + self.pipe.red_img.deltaY
                return 'locked'

            elif(self.pipe.red_img.detected and self.pipe.green_img.detected):
                rospy.loginfo("Centering")
                move_x = self.pipe.red_img.deltaX*self.con.mult_x
                rospy.loginfo("SM X: " + str(move_x))
                self.pipe.sendMovement(sidemove=move_x, wait=False)
                return 'check'
        else:
            rospy.loginfo("Cannot see both red and green")
            while self.pipe.green_img.detected and not self.pipe.isAborted:
                green_center = self.pipe.green_img.centroid[0]
                if self.pipe.red_img.detected:
                    return 'check'
                elif not self.pipe.green_img.ends:
                    move_x = (self.con.center_w - green_center)*self.con.mult_x/(self.con.screen_w)
                    rospy.loginfo("SM X: " + str(move_x))
                    self.pipe.sendMovement(sidemove=move_x, wait=False)
                    continue
            return 'aborted'
            
class Moving(smach.State):
    def __init__(self, pipe):
        smach.State.__init__(self, outcomes=['success', 'return', 'confirm', 'aborted'])
        self.pipe = pipe
        self.con = Config()

    def execute(self, userdata):
        while not self.pipe.isAborted:
            if self.pipe.red_img.detected:
                while self.pipe.red_img.detected:
                    if self.pipe.red_img.incline == "left":
                        self.pipe.sendMovement(sidemove=-0.5, wait=False)
                    else:
                        self.pipe.sendMovement(sidemove=0.5, wait=False)
                return 'success'
            else:
                rospy.loginfo("Lose track of green pole")
                self.pipe.sendMovement(forward=-2.0)
                return 'return'
        return 'aborted'
                
        
            
        
class Strike(smach.State):
    def __init__(self, pipe):
        smach.State.__init__(self, outcomes=['completed', 'back', 'aborted', 'circle'])
        self.pipe = pipe
        self.con = Config()

    def execute(self, userdata):
        strike = self.pipe.strike_distance
        self.pipe.setDepth = self.pipe.currDepth - 0.3
        self.pipe.sendMovement(forward=1.0)
        if self.pipe.red_img.detected:
            return 'back'
        else:
            self.pipe.isDone = True
            rospy.loginfo("Elapsed time: " + str((time.time() - self.pipe.initTime)/60.0))
            return 'completed'
        
class Circle(smach.State):
    def __init__(self, pipe):
        smach.State.__init__(self, outcomes=['completed'])
        self.pipe = pipe
        self.con = Config()

    def execute(self, userdata):
        self.pipe.sendMovement(sidemove=-0.6)
        self.pipe.sendMovement(forward=-1.0)
        self.pipe.sendMovement(sidemove= 0.6)
        self.pipe.sendMovement(forward=1.0)
        self.pipe.isDone = True
        return 'completed'
        
        

def main():
    rospy.init_node("PipeMaster")
    #rospy.loginfo("Init")
    pipe = Comms()
    
    sm = smach.StateMachine(outcomes=['completed', 'failed'])
    Inserver = smach_ros.IntrospectionServer("/pole", sm, "/pole")
    Inserver.start()

    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(pipe), 
                                transitions={'init':'SEARCH', 'aborted':'failed', 'test':'TEST'})
        smach.StateMachine.add("SEARCH", Search(pipe), 
                                transitions={'lost':'DISENGAGE', 'detected':'POSITION', 'aborted':'DISENGAGE'})
        smach.StateMachine.add("POSITION", Position(pipe), 
                                transitions={'locked':'MOVING', 'check':'POSITION', 'find':'SEARCH', 'aborted':'DISENGAGE'})
        smach.StateMachine.add("MOVING", Moving(pipe), 
                                transitions={'success':'STRIKE', 'return':'POSITION', 'confirm':'MOVING', 'aborted':'DISENGAGE'})
        smach.StateMachine.add("STRIKE", Strike(pipe), 
                                transitions={'completed':'DISENGAGE', 'back':'STRIKE', 'aborted':'DISENGAGE', 'circle':'CIRCLE'})
        smach.StateMachine.add("TEST", Test(pipe), 
                                transitions={'done':'TEST'})
        smach.StateMachine.add("CIRCLE", Circle(pipe), 
                                transitions={'completed':'DISENGAGE'})

    outcomes = sm.execute()
    rospy.spin()
    
