#!/usr/bin/env/python

'''
Pegs states
'''

import roslib; roslib.load_manifest('vision')
import rospy 

import time
import smach, smach_ros

from comms import Comms

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from dynamic_reconfigure.server import Server

from vision import PegsVision

#Globals
locomotionGoal = None

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['start_complete', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.3))
        
        if self.comms.isAlone:
            self.comms.register()
            rospy.loginfo("Starting Pegs")
        
        return 'start_complete'
    
class SearchPegs(smach.State):
    timeout = 1000
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['searchPeg_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):
        start = time.time()
        
        while not self.comms.foundSomething: 
            if self.comms.isKilled:
                return 'killed'
            if self.comms.isAborted or (time.time() - start) > self.timeout:
                self.comms.isAborted = True
                return 'aborted' 
            
            # Search in figure of 8? 
            self.comms.sendMovement(forward=0.2)
            rospy.sleep(rospy.Duration(0.3))   

        return 'searchPeg_complete'

class FollowSonar(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['sonar_complete', 'following_sonar', 'aborted', 'killed'])
        self.comms = comms
        
        self.comms.registerSonar()
        rospy.sleep(duration=0.5)
        
    def execute(self, ud):
        if self.comms.sonarDist > 2:
            self.comms.sendMovement(forward=self.comms.sonarDist,
                                    heading=self.comms.sonarBearing,
                                    timeout=0.5, blocking=False)
            return 'following_sonar'
        else:
            return 'sonar_complete'

class MoveForward(smach.State):
    counter = 0
    deltaXMult = 5.0
    deltaYMult = 0.2
    forward_setpoint = 0.3
    areaRectComplete = 500
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['forwarding', 'forward_complete', 'lost', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):    
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            self.comms.isAborted = True
            return 'aborted' 
        
        if not self.comms.foundSomething:
            self.counter = self.counter + 1
            if self.counter == 100:
                return 'lost'
        
        if self.comms.areaRect > self.areaRectComplete:
            self.comms.centering = True
            return 'forward_complete'
        
        # Forward and sidemove, keep heading
        if abs(self.comms.deltaY) > 0.010:
            self.comms.defaultDepth = self.comms.defaultDepth + self.comms.deltaY*self.deltaYMult
            if self.comms.defaultDepth < 0.1:
                self.comms.defaultDepth = 2.0
        
        self.comms.sendMovement(forward=self.forward_setpoint,
                                sidemove=self.comms.deltaX * self.deltaXMult,
                                depth=self.defaultDepth, timeout=0.4,
                                blocking=False)
        return 'forwarding'
    
class Centering(smach.State):
    counter = 0
    deltaXMult = 3.0
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'lost', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):        
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            self.comms.isAborted = True
            return 'aborted' 
        
        if not self.comms.foundSomething:
            self.counter = self.counter + 1
            if self.counter == 100:
                return 'lost'
        
        if self.comms.deltaX < 0.005:
            return 'centering_complete'
        
        # Center by sidemove
        self.comms.sendMovement(forward = 0.0,
                                sidemove = self.comms.deltaX*self.deltaXMult,
                                blocking = False)
        return 'centering'

class Offset(smach.State):    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['offset_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):        
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            self.comms.isAborted = True
            return 'aborted' 
        
        # Offset robot to move peg
        self.comms.sendMovement(sidemove = -0.20, blocking=True)
        return 'offset_complete'
        

class MovePeg(smach.State):    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['task_complete', 'move_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):
        start = time.time()
        
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            self.comms.isAborted = True
            return 'aborted' 
        
        grabberPub = rospy.Publisher("/manipulators", manipulator)
        
        # Grab red peg
        self.comms.grabRedPeg()
        
        # Move back a little
        self.comms.sendMovement(forward=-0.2, blocking=True)
        
        # Go forward a littled
        self.comms.sendMovement(forward=0.2, blocking=True)
        
        # Put red peg back
        self.comms.putPeg()
        
        # Move back to see the whole board again
        self.comms.sendMovement(forward=-0.5, blocking=True)
        
        # Reset variables
        self.comms.centering = False
        self.comms.count = self.comms.count + 1
        self.comms.centroidToPick = None

        if self.comms.count == 4:
            return 'task_complete'
        
        return 'move_complete'

def main():
    rospy.init_node('pegs_node', anonymous=False)
    rosRate = rospy.Rate(20)
    myCom = Comms()

    rospy.loginfo("Pegs Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={'start_complete': "SEARCHPEGS",
                                             'killed': 'killed'})
        
        smach.StateMachine.add("FOLLOWSONAR", FollowSonar(myCom),
                               transitions={'following_sonar': "FOLLOWSONAR",
                                           'sonar_complete': "SEARCHPEGS",
                                           'aborted': 'aborted',
                                           'killed': 'killed'})        
        
        smach.StateMachine.add("SEARCHPEGS", SearchPegs(myCom),
                                transitions={'searchPeg_complete': "MOVEFORWARD",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})       
    
        smach.StateMachine.add("MOVEFORWARD", MoveForward(myCom),
                                transitions={'forwarding': "MOVEFORWARD",
                                             'forward_complete': "CENTERING",
                                             'lost': "SEARCHPEGS",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})       
        
        smach.StateMachine.add("CENTERING", Centering(myCom),
                                transitions={'centering': "CENTERING",
                                             'centering_complete': "OFFSET",
                                             'lost': "SEARCHPEGS",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})        
 
        smach.StateMachine.add("OFFSET", Offset(myCom),
                                transitions={'offset_complete': "MOVEPEG",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})      
        
        smach.StateMachine.add("MOVEPEG", MovePeg(myCom),
                                transitions={'move_complete': "SEARCHPEGS",
                                             'task_complete': 'succeeded',
                                             'aborted': 'aborted',
                                             'killed': 'killed'})      
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/PEGS')
    introServer.start()
    
    sm.execute()
