import time

import rospy
import smach, smach_ros

from comms import Comms
from vision import PickupVision
from utils.utils import Utils

""" The entry script and smach StateMachine for the task"""

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['started', 'killed'])
        self.comms = comms

    def execute(self, userdata):
        self.comms.unregister()
        self.comms.visionMode = PickupVision.SITE

        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.3))

        self.comms.register()
        if self.comms.isAlone:
            rospy.sleep(rospy.Duration(1))
            self.comms.inputHeading = self.comms.curHeading
            self.comms.sendMovement(h=self.comms.inputHeading,
                                    d=self.comms.defaultDepth,
                                    blocking=True)
        self.comms.retVal = None
        return 'started'

class SearchSite(smach.State):
    timeout = 45

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['aborted', 'timeout', 'foundSite'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or \
           self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        start = time.time()

        while not self.comms.retVal or \
              len (self.comms.retVal['site']) < 1:
            if time.time() - start > self.timeout:
                self.comms.failTask()
                return 'timeout'
            if self.comms.isAborted:
                return 'aborted'
            #self.comms.sendMovement(f=1.0, d=self.comms.defaultDepth,
            #                        blocking=False)
            rospy.sleep(rospy.Duration(0.1))

        return 'foundSite'

class CenterSite(smach.State):
    width = PickupVision.screen['width']
    height = PickupVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0

    maxdx = 0.03
    maxdy = 0.03
    xcoeff = 2.5
    ycoeff = 3.0

    numTrials = 1
    trialPassed = 0

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centered',
                                             'centering',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or \
           self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        if not self.comms.retVal or \
           len(self.comms.retVal['site']) < 1:
            self.trialPassed = 0
            return 'lost'

        site = self.comms.retVal['site']

        dx = (site['centroid'][0] - self.centerX) / self.width
        dy = (site['centroid'][1] - self.centerY) / self.height

        if abs(dx) < self.maxdx and abs(dy) < self.maxdy:
            self.comms.motionClient.cancel_all_goals()
            if self.trialPassed == self.numTrials:
                self.trialPassed = 0
                self.comms.visionMode = PickupVision.SAMPLES
                self.comms.notifyCentered()
                self.comms.sendMovement(h=self.comms.inputHeading,
                                        d=self.comms.sinkingDepth,
                                        blocking=True)
                return 'centered'
            else:
                self.trialPassed += 1
                return 'centering'

        self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                d=self.comms.defaultDepth,
                                h=self.comms.inputHeading,
                                blocking=False)
        return 'centering'
        

class Search(smach.State):
    timeout = 3
    curPattern = 0
    pattern = ((0, 0), (0.4, -0.4), (0.4, 0.4), (-0.4, -0.4), (-0.4, 0.4)) 
    lastPattern = len(pattern) - 1

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['foundSamples',
                                             'searching',
                                             'timeout',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or \
           self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        self.comms.sendMovement(f=self.pattern[self.curPattern][0],
                                sm=self.pattern[self.curPattern][1],
                                h=self.comms.inputHeading,
                                d=self.comms.sinkingDepth,
                                timeout=5,
                                blocking=False)
        start = time.time()
        while not self.comms.retVal or \
              len (self.comms.retVal['samples']) < 1:
            if time.time() - start > self.timeout:
                if self.curPattern == self.lastPattern:
                    self.curPattern = 0
                    self.comms.failTask()
                    return 'timeout'
                else:
                    self.comms.sendMovement(f=-self.pattern[self.curPattern][0],
                                            sm=-self.pattern[self.curPattern][1],
                                            h=self.comms.inputHeading,
                                            d=self.comms.sinkingDepth,
                                            timeout=5,
                                            blocking=False)
                    self.curPattern += 1
                    return 'searching'
            if self.comms.isAborted or self.comms.isKilled:
                self.curPattern = 0
                return 'aborted'
            rospy.sleep(rospy.Duration(0.3))

        self.curPattern = 0
        return 'foundSamples'

class Center(smach.State):
    width = PickupVision.screen['width']
    height = PickupVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0

    maxdx = 0.03
    maxdy = 0.03
    xcoeff = 2.5
    ycoeff = 2.0

    numTrials = 1
    trialPassed = 0

    lostTimeout = 5

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centered',
                                             'centering',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or \
           self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        start = time.time()
        while not self.comms.retVal or \
           len(self.comms.retVal['samples']) < 1:
            if self.comms.isKilled or self.comms.isAborted:
                self.comms.abortMission()
                return 'aborted'
            if time.time() - start > self.lostTimeout:
                self.trialPassed = 0
                return 'lost'
            else:
                rospy.sleep(rospy.Duration(0.1))

        samples = self.comms.retVal['samples']
        closest = min(samples,
                      key=lambda c:
                      Utils.distBetweenPoints(c['centroid'],
                                              (self.centerX, self.centerY)))

        dx = (closest['centroid'][0] - self.centerX) / self.width
        dy = (closest['centroid'][1] - self.centerY) / self.height

        if abs(dx) < self.maxdx and abs(dy) < self.maxdy:
            self.comms.motionClient.cancel_all_goals()
            if self.trialPassed == self.numTrials:
                self.trialPassed = 0
                self.comms.selectedSample = closest
                return 'centered'
            else:
                self.trialPassed += 1
                return 'centering'

        self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                d=self.comms.sinkingDepth,
                                h=self.comms.inputHeading,
                                blocking=False)
        return 'centering'

class Align(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['aligned',
                                             'aligning',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        if not self.comms.retVal or \
           len(self.comms.retVal['samples']) == 0:
            return 'lost'

        self.comms.adjustHeading = self.comms.inputHeading

        # Align with the samples
        #dAngle = Utils.toHeadingSpace(self.comms.selectedSample['angle'])
        #adjustAngle = Utils.normAngle(dAngle + self.comms.curHeading)
        #self.comms.adjustHeading = adjustAngle
        #self.comms.sendMovement(h=adjustAngle,
        #                        d=self.comms.sinkingDepth,
        #                        blocking=False)

        return 'aligned'


class Center2(smach.State):
    width = PickupVision.screen['width']
    height = PickupVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0

    maxdx = 0.03
    maxdy = 0.03
    xcoeff = 2.5
    ycoeff = 2.0

    numTrials = 1
    trialPassed = 0

    lostTimeout = 5

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centered',
                                             'centering',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or \
           self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        start = time.time()
        while not self.comms.retVal or \
           len(self.comms.retVal['samples']) < 1:
            if self.comms.isKilled or self.comms.isAborted:
                self.comms.abortMission()
                return 'aborted'
            if time.time() - start > self.lostTimeout:
                self.trialPassed = 0
                return 'lost'
            else:
                rospy.sleep(rospy.Duration(0.1))

        samples = self.comms.retVal['samples']
        closest = min(samples,
                      key=lambda c:
                      Utils.distBetweenPoints(c['centroid'],
                                              (self.centerX, self.centerY)))

        dx = (closest['centroid'][0] - self.centerX) / self.width
        dy = (closest['centroid'][1] - self.centerY) / self.height

        if abs(dx) < self.maxdx and abs(dy) < self.maxdy:
            self.comms.motionClient.cancel_all_goals()
            if self.trialPassed == self.numTrials:
                self.trialPassed = 0
                return 'centered'
            else:
                self.trialPassed += 1
                return 'centering'

        self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                d=self.comms.sinkingDepth,
                                h=self.comms.adjustHeading,
                                blocking=False)
        return 'centering'
    
class Approach(smach.State):
    width = PickupVision.screen['width']
    height = PickupVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0

    xcoeff = 2.2
    ycoeff = 1.8

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['completed',
                                             'approaching',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        if not self.comms.retVal or \
           len(self.comms.retVal['samples']) == 0:
            self.comms.adjustDepth = self.comms.curDepth
            rospy.sleep(rospy.Duration(0.1))
            return 'approaching'

        curArea = self.comms.retVal['samples'][0]['area']
        rospy.loginfo("Area: {}".format(curArea))
        if curArea  > self.comms.grabbingArea or \
           self.comms.curDepth > self.comms.grabbingDepth:
            self.comms.adjustDepth = self.comms.curDepth
            return 'completed'

        samples = self.comms.retVal['samples']
        closest = min(samples,
                      key=lambda c:
                      Utils.distBetweenPoints(c['centroid'],
                                              (self.centerX, self.centerY)))
        dx = (closest['centroid'][0] - self.centerX) / self.width
        dy = (closest['centroid'][1] - self.centerY) / self.height

        self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                d=self.comms.curDepth + 0.1,
                                h=self.comms.adjustHeading,
                                timeout=2,
                                blocking=False)
        return 'approaching'

class Center3(smach.State):
    width = PickupVision.screen['width']
    height = PickupVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0 + 40

    maxdx = 0.03
    maxdy = 0.03
    xcoeff = 2.5
    ycoeff = 1.0

    numTrials = 1
    trialPassed = 0

    lostTimeout = 5

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centered',
                                             'centering',
                                             'lost',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or \
           self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        start = time.time()
        while not self.comms.retVal or \
           len(self.comms.retVal['samples']) < 1:
            if self.comms.isKilled or self.comms.isAborted:
                self.comms.abortMission()
                return 'aborted'
            if time.time() - start > self.lostTimeout:
                self.trialPassed = 0
                return 'lost'
            else:
                rospy.sleep(rospy.Duration(0.1))

        samples = self.comms.retVal['samples']
        closest = min(samples,
                      key=lambda c:
                      Utils.distBetweenPoints(c['centroid'],
                                              (self.centerX, self.centerY)))

        dx = (closest['centroid'][0] - self.centerX) / self.width
        dy = (closest['centroid'][1] - self.centerY) / self.height

        if abs(dx) < self.maxdx and abs(dy) < self.maxdy:
            self.comms.motionClient.cancel_all_goals()
            if self.trialPassed == self.numTrials:
                self.trialPassed = 0
                return 'centered'
            else:
                self.trialPassed += 1
                return 'centering'

        self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                d=self.comms.adjustDepth,
                                h=self.comms.adjustHeading,
                                blocking=False)
        return 'centering'


class Grab(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['grabbed',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        self.comms.activateVelocity()
        self.comms.sendMovement(f=0.1,
                                d=self.comms.lastDepth,
                                h=self.comms.adjustHeading,
                                timeout=6,
                                blocking=False)
        for i in range(5):
            self.comms.grab()
            rospy.sleep(rospy.Duration(0.2))

        return 'grabbed'


class Surface(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['completed',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        self.comms.deactivateVelocity()
        self.comms.sendMovement(d=self.comms.defaultDepth,
                                h=self.comms.adjustHeading,
                                blocking=True)

        self.comms.taskComplete()
        return 'completed'


def main():
    rospy.init_node('pickup')
    myCom = Comms()    

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])
    with sm:
        smach.StateMachine.add('DISENGAGE',
                               Disengage(myCom),
                               transitions={'started':'SEARCHSITE',
                                            'killed':'killed'})
        smach.StateMachine.add('SEARCHSITE',
                               SearchSite(myCom),
                               transitions={'foundSite':'CENTERSITE',
                                            'timeout':'DISENGAGE',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('CENTERSITE',
                               CenterSite(myCom),
                               transitions={'centered':'SEARCH',
                                            'centering':'CENTERSITE',
                                            'lost':'SEARCHSITE',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('SEARCH',
                               Search(myCom),
                               transitions={'foundSamples':'CENTER',
                                            'searching':'SEARCH',
                                            'timeout':'DISENGAGE',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('CENTER',
                               Center(myCom),
                               transitions={'centered':'ALIGN',
                                            'centering':'CENTER',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('ALIGN',
                               Align(myCom),
                               transitions={'aligned': 'CENTER2',
                                            'aligning': 'ALIGN',
                                            'lost': 'SEARCH',
                                            'aborted': 'DISENGAGE'})
        smach.StateMachine.add('CENTER2',
                               Center2(myCom),
                               transitions={'centered':'APPROACH',
                                            'centering':'CENTER2',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('APPROACH',
                               Approach(myCom),
                               transitions={'completed':'CENTER3',
                                            'approaching':'APPROACH',
                                            'lost':'SEARCH',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('CENTER3',
                               Center3(myCom),
                               transitions={'centered':'GRAB',
                                            'centering':'CENTER3',
                                            'lost':'GRAB',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('GRAB',
                               Grab(myCom),
                               transitions={'grabbed':'SURFACE',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('SURFACE',
                               Surface(myCom),
                               transitions={'completed':'DISENGAGE',
                                            'aborted':'DISENGAGE'})

    introServer = smach_ros.IntrospectionServer('mission_server',
                                                sm,
                                                '/MISSION/PICKUP')
    introServer.start()

    try:
        sm.execute()
    except Exception as e:
        rospy.logerr(str(e))
        myCom.failTask()
    finally:
        rospy.signal_shutdown("bins task ended")
