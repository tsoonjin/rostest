import rospy
from dynamic_reconfigure.server import Server as DynServer

from bbauv_msgs.srv import mission_to_vision, vision_to_mission, \
        mission_to_visionResponse
from bbauv_msgs.msg import controller, manipulator, depth
from utils.config import pickupConfig as Config

from vision import PickupVision
from bot_common.bot_comms import GenericComms

class Comms(GenericComms):
    """ Class to facilitate communication b/w ROS and task submodules """
    def __init__(self, taskMode='pickup'):
        GenericComms.__init__(self, PickupVision(self))
        self.curDepth = 0.0
        self.taskMode = taskMode

        if taskMode == 'pickup':
            self.defaultDepth = 0.2
            self.sinkingDepth = 2.6 #0.8 #1.9 #2.0 #robosub 8.9
            self.grabbingDepth = 2.8 #0.9 #2.0 #3.2 #robosub 9
            self.lastDepth = 3.8 #2.1 #2.9 #3.8 #robosub 9.9
            self.grabbingArea = 20000

            self.visionMode = PickupVision.SITE
        elif taskMode == 'drop':
            self.defaultDepth = 0.2 #1.2
            self.sinkingDepth = 0.8 #1.5 #2.5
            self.droppingDepth = 1.7

            self.visionMode = PickupVision.BOX
            self.calledCount = 0

        self.depthSub = rospy.Subscriber("/depth", depth, self.depthCb)
        self.maniPub = rospy.Publisher("/manipulators", manipulator)
        self.dynServer = DynServer(Config, self.reconfigure)

        if not self.isAlone:
            self.initComms(taskMode)

    def initComms(self, name):
        # Initialize mission planner communication server and client
        self.comServer = rospy.Service("/{}/mission_to_vision".format(name),
                                       mission_to_vision,
                                       self.handleSrv)
        rospy.loginfo("Waiting for vision to mission service")
        self.toMission = rospy.ServiceProxy("/{}/vision_to_mission".format(name),
                                            vision_to_mission)
        self.toMission.wait_for_service()

    def abortMission(self):
        if self.taskMode == 'drop':
            pass
            #self.drop()
            #if self.calledCount == 1:
            #    self.sendMovement(d=-0.5,
            #                      h=self.inputHeading,
            #                      timeout=5,
            #                      blocking=False)
        GenericComms.abortMission(self)

    def failTask(self):
        if self.taskMode == 'drop':
            pass
            #self.drop()
            #if self.calledCount == 1:
            #    self.sendMovement(d=-0.5,
            #                      h=self.inputHeading,
            #                      timeout=5,
            #                      blocking=False)
        GenericComms.failTask(self)

    def depthCb(self, data):
        self.curDepth = data.depth

    def handleSrv(self, req):
        if req.start_request:
            rospy.loginfo("Received Start Request")
            self.isAborted = False
            self.defaultDepth = req.start_ctrl.depth_setpoint + 0.2
            self.inputHeading = req.start_ctrl.heading_setpoint
            if self.taskMode == 'drop':
                self.calledCount += 1
            return mission_to_visionResponse(start_response=True,
                                             abort_response=False,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))
        elif req.abort_request:
            rospy.loginfo("Received Abort Request")
            self.motionClient.cancel_all_goals()
            self.isAborted = True
            while not self.abortedDone:
                rospy.sleep(rospy.Duration(0.3))
                self.motionClient.cancel_all_goals()
            return mission_to_visionResponse(start_response=False,
                                             abort_response=True,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))

    def reconfigure(self, config, level):
        rospy.loginfo("Received reconfigure request")
        self.params = {'yellowLoThresh': (config.yellowLoH,
                                          config.yellowLoS,
                                          config.yellowLoV),
                       'yellowHiThresh': (config.yellowHiH,
                                          config.yellowHiS,
                                          config.yellowHiV),
                       'yellowLoThresh2': (config.yellowLoH2,
                                          config.yellowLoS2,
                                          config.yellowLoV2),
                       'yellowHiThresh2': (config.yellowHiH2,
                                          config.yellowHiS2,
                                          config.yellowHiV2),

                       'greenLoThresh': (config.greenLoH,
                                         config.greenLoS,
                                         config.greenLoV),
                       'greenHiThresh': (config.greenHiH,
                                         config.greenHiS,
                                         config.greenHiV),
                       'redLoThresh1': (config.redLoH1,
                                        config.redLoS1,
                                        config.redLoV1),
                       'redHiThresh1': (config.redHiH1,
                                        config.redHiS1,
                                        config.redHiV1),
                       'redLoThresh2': (config.redLoH2,
                                        config.redLoS1,
                                        config.redLoV1),
                       'redHiThresh2': (config.redHiH2,
                                        config.redHiS1,
                                        config.redHiV1),

                       'minSiteArea': config.minSiteArea,
                       'minContourArea' : config.minArea,
                       'maxContourArea' : config.maxArea}
        self.grabbingArea = config.grabbingArea
        self.visionFilter.updateParams()
        return config

    def grab(self):
        self.maniPub.publish(4)

    def drop(self):
        self.maniPub.publish(0)


def main():
    pass
    # testCom = Comms()
