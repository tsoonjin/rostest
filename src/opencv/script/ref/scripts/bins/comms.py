import rospy
from dynamic_reconfigure.server import Server as DynServer

from bbauv_msgs.msg import manipulator, controller
from bbauv_msgs.srv import mission_to_visionResponse

from utils.config import binsConfig as Config

from vision import BinsVision
from bot_common.bot_comms import GenericComms

class Comms(GenericComms):
    """ Class to facilitate communication b/w ROS and task submodules """

    def __init__(self):
        GenericComms.__init__(self, BinsVision(self))
        self.defaultDepth = 0.2 #0.2
        self.aligningDepth = 1.4 #0.2 #0.8 #1.9 #robosub 9.4
        self.sinkingDepth = 3.0 #2.0 #2.7 #robosub 9.6
        self.search2Depth = 0.5 #0.2 #1.2 #robosub 8
        self.turnDepth = 0.5 #0.8 #1.9 #robosub 8.2

        self.dynServer = DynServer(Config, self.reconfigure)
        self.maniPub = rospy.Publisher("/manipulators", manipulator)

        if not self.isAlone:
            self.initComms("bins")

    def handleSrv(self, req):
        if req.start_request:
            rospy.loginfo("Received Start Request")
            self.isAborted = False
            self.defaultDepth = req.start_ctrl.depth_setpoint
            self.inputHeading = req.start_ctrl.heading_setpoint
            return mission_to_visionResponse(start_response=True,
                                             abort_response=False,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))
        elif req.abort_request:
            rospy.loginfo("Received Abort Request!!!")
            self.motionClient.cancel_all_goals()
            self.isAborted = True
            while not self.abortedDone:
                rospy.sleep(rospy.Duration(0.3))
                self.motionClient.cancel_all_goals()
            return mission_to_visionResponse(start_response=False,
                                             abort_response=True,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))

    def abortMission(self):
        self.drop()
        GenericComms.abortMission(self)

    def failTask(self):
        self.drop()
        GenericComms.failTask(self)

    def drop(self):
        self.maniPub.publish(8)

    def reconfigure(self, config, level):
        rospy.loginfo("Received dynamic_reconfigure")
        self.params = {'hsvLoThresh1' : (config.loH1, config.loS1, config.loV1),
                       'hsvHiThresh1' : (config.hiH1, config.hiS1, config.hiV1),
                       'hsvLoThresh2' : (config.loH2, config.loS1, config.loV1),
                       'hsvHiThresh2' : (config.hiH2, config.hiS1, config.hiV1),
                       'minContourArea' : config.alienMinArea,
                       'adaptiveCoeff' : config.adaptiveCoeff,
                       'adaptiveOffset' : config.adaptiveOffset,
                       'areaThresh' : config.binMinArea,
                       'matchBound': config.matchBound,
                       'ratioBound' : config.ratioBound,
                       'blackThresh' : config.blackThresh,
                       'epsilon': config.epsilon}
        self.visionFilter.updateParams()
        return config

def main():
    testCom = Comms()
