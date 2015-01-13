#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import rospy

import smach
import math

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from bucket_vision import BucketDetector

#Starts off in disengage class
class Disengage(smach.State):
    timeout = 1500

    def __init__(self, bucketDetector):
        smach.State.__init__(self, outcomes=['start_complete', 'task_complete', 'aborted'])
        self.bucketDetector = bucketDetector

    def execute(self, userdata):
        self.bucketDetector.unregister()
        #Check for abort or kill signal
        timecount = 0
        while bucketDetector.isAborted:
            if timecount > self.timeout or bucketDetector.isKilled:
                return 'aborted'
            rospy.sleep(rospy.Duration(0.1))
            timecount += 1

        self.bucketDetector.register()
        return 'start_complete'

#Searches for the bucket
class Searching1(smach.State):
    def __init__(self, bucketDetector):
        smach.State.__init__(self, outcomes=['search_complete', 'aborted'])
        self.bucketDetector = bucketDetector

    def execute(self, userdata):
        #Check for abort signal
        if self.bucketDetector.isAborted:
            return 'aborted'

        while not self.bucketDetector.rectData['detected']:
            if self.bucketDetector.isAborted:
                return 'aborted'
            rospy.sleep(rospy.Duration(0.1))

        self.bucketDetector.searchComplete()

        self.bucketDetector.depth_setpoint = 0.55
        self.bucketDetector.stopRobot()
        rospy.sleep(rospy.Duration(2.5))

        return 'search_complete'

class Searching2(smach.State):
    timeout = 30

    def __init__(self, bucketDetector):
        smach.State.__init__(self, outcomes=['search_complete', 'aborted'])
        self.bucketDetector = bucketDetector

    def execute(self, userdata):
        if self.bucketDetector.isAborted:
            return 'aborted'

        timecount = 0
        while not self.bucketDetector.rectData['detected']:
            if timecount > self.timeout:
                break
            timecount += 1
            rospy.sleep(rospy.Duration(0.1))

#         while self.bucketDetector.revertMovement():
#             if self.bucketDetector.rectData['detected']:
#                 return 'search_complete'

        timecount = 0
        while not self.bucketDetector.rectData['detected']:
            if timecount > self.timeout:
                self.bucketDetector.abortMission()
                return 'aborted'
            timecount += 1
            rospy.sleep(rospy.Duration(0.1))

        return 'search_complete'

#Centers the robot to the bucket
class Centering(smach.State):
    def __init__(self, bucketDetector):
        smach.State.__init__(self, outcomes=['centering_complete', 'centering',
                                             'lost_bucket', 'aborted'])
        self.bucketDetector = bucketDetector

    def execute(self, userdata):
        if self.bucketDetector.isAborted:
            return 'aborted'

        rectData = self.bucketDetector.rectData
        if not rectData['detected']:
            return 'lost_bucket'

        screenWidth = self.bucketDetector.screen['width']
        screenCenterX = screenWidth / 2
        screenHeight = self.bucketDetector.screen['height']
        screenCenterY = screenHeight / 2
        deltaX = (rectData['centroid'][0] - screenCenterX) / screenWidth
        deltaY = (rectData['centroid'][1] - screenCenterY) / screenHeight
        rospy.loginfo("x-off: %lf, y-off: %lf", deltaX, deltaY)

        if abs(deltaX) < 0.02 and abs(deltaY) < 0.02:
            self.bucketDetector.stopRobot()
            rospy.loginfo("--- Done centering! ---")
            return 'centering_complete'

        fwd_setpoint = math.copysign(1.5 * abs(deltaY), -deltaY)
        sm_setpoint = math.copysign(2.0 * abs(deltaX), deltaX)
        self.bucketDetector.sendMovement(f=fwd_setpoint, sm=sm_setpoint)
        return 'centering'

#Fire the gold ball
class Firing(smach.State):
    def __init__(self, bucketDetector):
        smach.State.__init__(self, outcomes=['firing_complete', 'aborted'])
        self.bucketDetector = bucketDetector

    def execute(self, userdata):
        rospy.loginfo("--- Moving down and backward! ---")
        self.bucketDetector.depth_setpoint = 0.8
        self.bucketDetector.sendMovementBlocking(f=-0.08)
        self.bucketDetector.depth_setpoint = 0.5

        rospy.loginfo("--- Shooting!!! ---")
        firePub = rospy.Publisher("/manipulators", manipulator)
        for i in range(10):
            firePub.publish(self.bucketDetector.maniData | 1)
            rospy.sleep(rospy.Duration(0.1))

        self.bucketDetector.stopRobot()
        rospy.sleep(rospy.Duration(1))
        for i in range(10):
            firePub.publish(self.bucketDetector.maniData & 0)
            rospy.sleep(rospy.Duration(0.1))

        self.bucketDetector.taskComplete()
        return 'firing_complete'

if __name__ == '__main__':
    rospy.init_node('bucketdetector')
    bucketDetector = BucketDetector()
    rospy.loginfo("Bucket loaded!")

    sm = smach.StateMachine(outcomes=['task_complete', 'aborted'])
    with sm:
        smach.StateMachine.add('DISENGAGE', Disengage(bucketDetector),
                               transitions={'start_complete' : 'SEARCHING1',
                                            'aborted' : 'aborted',
                                            'task_complete' : 'task_complete'})
        smach.StateMachine.add('SEARCHING1', Searching1(bucketDetector),
                               transitions={'search_complete' : 'CENTERING',
                                            'aborted' : 'DISENGAGE'})
        smach.StateMachine.add('SEARCHING2', Searching2(bucketDetector),
                               transitions={'search_complete' : 'CENTERING',
                                            'aborted' : 'DISENGAGE'})
        smach.StateMachine.add('CENTERING', Centering(bucketDetector),
                               transitions={'centering_complete' : 'FIRING',
                                            'centering' : 'CENTERING',
                                            'lost_bucket':'SEARCHING2',
                                            'aborted' : 'DISENGAGE'})
        smach.StateMachine.add('FIRING', Firing(bucketDetector),
                               transitions={'firing_complete' : 'task_complete',
                                            'aborted':'DISENGAGE'})

    sis = smach_ros.IntrospectionServer('flare_task', sm, '/SM_ROOT')
    sis.start()
    outcomes = sm.execute()
    rospy.loginfo(outcomes)
