import time

import rospy
import smach, smach_ros

from comms import Comms
from vision import PickupVision

""" States for finding the yellow box and dropping the samples """

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['started', 'killed'])
        self.comms = comms

    def execute(self, userdata):
        self.comms.unregister()
        self.comms.visionMode = PickupVision.BOX

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

class SearchBox(smach.State):
    timeout = 10

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['aborted', 'timeout', 'foundBox'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or \
           self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        start = time.time()

        while not self.comms.retVal or \
              len (self.comms.retVal['box']) < 1:
            if time.time() - start > self.timeout:
                self.comms.failTask()
                return 'timeout'
            if self.comms.isAborted or self.comms.isKilled:
                return 'aborted'
            rospy.sleep(rospy.Duration(0.1))

        self.comms.sendMovement(h=self.comms.inputHeading,
                                d=self.comms.defaultDepth,
                                blocking=True)
        return 'foundBox'

class CenterBox(smach.State):
    width = PickupVision.screen['width']
    height = PickupVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0

    maxdx = 0.03
    maxdy = 0.03
    xcoeff = 3.0
    ycoeff = 2.5

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
           len(self.comms.retVal['box']) < 1:
            if self.comms.isKilled or self.comms.isAborted:
                self.comms.abortMission()
                return 'aborted'
            if time.time() - start > self.lostTimeout:
                self.trialPassed = 0
                return 'lost'
            else:
                rospy.sleep(rospy.Duration(0.1))

        box = self.comms.retVal['box']

        dx = (box['centroid'][0] - self.centerX) / self.width
        dy = (box['centroid'][1] - self.centerY) / self.height

        if abs(dx) < self.maxdx and abs(dy) < self.maxdy:
            self.comms.motionClient.cancel_all_goals()
            if self.trialPassed == self.numTrials:
                self.comms.sendMovement(d=-0.5,
                                        h=self.comms.inputHeading,
                                        timeout=5,
                                        blocking=False)
                self.comms.sendMovement(d=self.comms.sinkingDepth,
                                        h=self.comms.inputHeading,
                                        blocking=True)
                self.trialPassed = 0
                self.comms.notifyCentered()
                return 'centered'
            else:
                self.trialPassed += 1
                return 'centering'

        self.comms.sendMovement(f=-self.ycoeff*dy, sm=self.xcoeff*dx,
                                d=self.comms.defaultDepth,
                                h=self.comms.inputHeading,
                                blocking=False)
        return 'centering'

class CenterBox2(smach.State):
    width = PickupVision.screen['width']
    height = PickupVision.screen['height']
    centerX = width / 2.0
    centerY = height / 2.0

    maxdx = 0.03
    maxdy = 0.03
    xcoeff = 3.0
    ycoeff = 2.5

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
           len(self.comms.retVal['box']) < 1:
            if self.comms.isKilled or self.comms.isAborted:
                self.comms.abortMission()
                return 'aborted'
            if time.time() - start > self.lostTimeout:
                self.trialPassed = 0
                return 'lost'
            else:
                rospy.sleep(rospy.Duration(0.1))

        box = self.comms.retVal['box']

        dx = (box['centroid'][0] - self.centerX) / self.width
        dy = (box['centroid'][1] - self.centerY) / self.height

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
                                h=self.comms.inputHeading,
                                blocking=False)
        return 'centering'

class Drop(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['completed',
                                             'aborted'])
        self.comms = comms

    def execute(self, userdata):
        if self.comms.isAborted or self.comms.isKilled:
            self.comms.abortMission()
            return 'aborted'

        self.comms.sendMovement(f=0.1,
                                h=self.comms.inputHeading,
                                d=self.comms.droppingDepth,
                                blocking=True)
        for i in range(5):
            self.comms.drop()
            rospy.sleep(rospy.Duration(0.3))

        self.comms.sendMovement(h=self.comms.inputHeading,
                                d=self.comms.defaultDepth,
                                blocking=True)
        self.comms.taskComplete()

        return 'completed'


def main():
    rospy.init_node('drop')
    myCom = Comms(taskMode='drop')

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])
    with sm:
        smach.StateMachine.add('DISENGAGE',
                               Disengage(myCom),
                               transitions={'started':'SEARCHBOX',
                                            'killed':'killed'})
        smach.StateMachine.add('SEARCHBOX',
                               SearchBox(myCom),
                               transitions={'timeout': 'DISENGAGE',
                                            'aborted': 'DISENGAGE',
                                            'foundBox': 'CENTERBOX'})
        smach.StateMachine.add('CENTERBOX',
                               CenterBox(myCom),
                               transitions={'centered': 'CENTERBOX2',
                                            'centering': 'CENTERBOX',
                                            'lost': 'SEARCHBOX',
                                            'aborted': 'DISENGAGE'})
        smach.StateMachine.add('CENTERBOX2',
                               CenterBox2(myCom),
                               transitions={'centered': 'DROP',
                                            'centering': 'CENTERBOX2',
                                            'lost': 'DROP',
                                            'aborted': 'DISENGAGE'})
        smach.StateMachine.add('DROP',
                               Drop(myCom),
                               transitions={'completed': 'DISENGAGE',
                                            'aborted': 'DISENGAGE'})

    introServer = smach_ros.IntrospectionServer('mission_server',
                                                sm,
                                                '/MISSION/DROP')
    introServer.start()

    try:
        sm.execute()
    except Exception as e:
        rospy.logerr(str(e))
        myCom.failTask()
    finally:
        rospy.signal_shutdown("bins task ended")
