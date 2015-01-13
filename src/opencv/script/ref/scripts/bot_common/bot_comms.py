import rospy
from sensor_msgs.msg import Image
import actionlib

from bbauv_msgs.msg import compass_data, \
    ControllerAction, ControllerGoal, controller
from bbauv_msgs.srv import mission_to_vision, vision_to_mission
from bbauv_msgs.srv import set_controller

from utils.utils import Utils
import utils.config as config

import signal


class GenericComms:
    """ Class to facilitate communication b/w ROS and task submodules """
    processingRate = 2
    processingCount = 0

    def __init__(self, visionFilter):
        signal.signal(signal.SIGINT, self.userQuit)

        # Initialize default values
        self.inputHeading = 0
        self.curHeading = 0
        self.retVal = None
        self.defaultDepth = 0.0

        # Initialize flags
        self.isAborted = True
        self.abortedDone = False
        self.isKilled = False
        self.canPublish = False

        # Initialize subscribers and publishers
        self.camSub = None
        self.compassSub = None
        self.outPub = None

        #Initialize vision filter
        self.visionFilter = visionFilter

        # Get private params
        self.isAlone = rospy.get_param('~alone', False)
        self.imageTopic = rospy.get_param('~image', config.botCamTopic)

        # Communicate with motion control server
        self.motionClient = actionlib.SimpleActionClient("LocomotionServer",
                                                         ControllerAction)
        try:
            rospy.loginfo("Waiting for LocomotionServer...")
            self.motionClient.wait_for_server(timeout=rospy.Duration(5))
        except:
            rospy.loginfo("LocomotionServer timeout!")
            self.isKilled = True
        self.setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)

        # Run straight away if in alone mode
        if self.isAlone:
            self.setServer(forward=True, sidemove=True, heading=True, depth=True,
                      pitch=True, roll=True, topside=False, navigation=False,
                      forward_vel=False, sidemove_vel=False)
            self.isAborted = False
            self.canPublish = True

    def initComms(self, taskName):
        # Initialize mission planner communication server and client
        rospy.loginfo("Starting /{}/mission_to_vision".format(taskName))
        self.comServer = rospy.Service("/{}/mission_to_vision".format(taskName),
                                       mission_to_vision,
                                       self.handleSrv)
        rospy.loginfo("Waiting for vision to mission service")
        self.toMission = rospy.ServiceProxy("/{}/vision_to_mission".format(taskName),
                                            vision_to_mission)
        self.toMission.wait_for_service()

    def register(self):
        self.camSub = rospy.Subscriber(self.imageTopic, Image, self.camCallback)
        self.compassSub = rospy.Subscriber(config.compassTopic,
                                           compass_data,
                                           self.compassCallback)
        self.outPub = rospy.Publisher(config.visionFilterTopic, Image)
        self.canPublish = True
        self.abortedDone = False

    def unregister(self):
        if self.camSub is not None:
            self.camSub.unregister()
        if self.compassSub is not None:
            self.compassSub.unregister()
        self.canPublish = False
        self.retVal = None
        self.abortedDone = True

    def camCallback(self, rosImg):
        try:
            if self.processingCount == self.processingRate:
                self.retVal, outImg = self.visionFilter.gotFrame(Utils.rosimg2cv(rosImg))
                if self.canPublish and outImg is not None:
                    self.outPub.publish(Utils.cv2rosimg(outImg))
                self.processingCount = 0
            self.processingCount += 1
        except Exception as e:
            print e

    def compassCallback(self, data):
        self.curHeading = data.yaw

    def userQuit(self, signal, frame):
        self.canPublish = False
        self.isAborted = True
        self.isKilled = True
        rospy.signal_shutdown("Task manually killed")

    def abortMission(self):
        rospy.loginfo("Stopping task")
        self.canPublish = False
        self.isAborted = True
        self.motionClient.cancel_all_goals()

    def failTask(self):
        rospy.loginfo("Sending fail request to mission planner")
        if not self.isAlone:
            self.toMission(fail_request=True, task_complete_request=False,
                           task_complete_ctrl=controller(
                               heading_setpoint=self.curHeading))
        self.canPublish = False
        self.isAborted = True
        self.motionClient.cancel_all_goals()

    def notifyCentered(self):
        rospy.loginfo("Sending centered request to mission planner")
        if not self.isAlone:
            self.toMission(fail_request=False, task_complete_request=False,
                           centered=True,
                           task_complete_ctrl=controller(
                               heading_setpoint=self.curHeading))

    def taskComplete(self, heading=0.0):
        rospy.loginfo("Sending Complete request to mission planner")
        if not self.isAlone:
            self.toMission(fail_request=False, task_complete_request=True,
                           task_complete_ctrl=controller(
                               heading_setpoint=heading))
        self.canPublish = False
        self.isAborted = True
        self.motionClient.cancel_all_goals()

    def searchComplete(self):
        if not self.isAlone:
            rospy.loginfo("Sending Complete request to mission planner")
            self.toMission(fail_request=False, task_complete_request=False,
                           search_request=True,
                           task_complete_ctrl=controller())

    def deactivateVelocity(self):
        self.setServer(forward=True, sidemove=True, heading=True, depth=True,
                       pitch=True, roll=True, topside=False, navigation=False,
                       forward_vel=False, sidemove_vel=False)

    def activateVelocity(self):
        self.setServer(forward=False, sidemove=False, heading=True, depth=True,
                       pitch=True, roll=True, topside=False, navigation=False,
                       forward_vel=True, sidemove_vel=True)

    def sendMovement(self, f=0.0, sm=0.0, h=None, d=None,
                     fv=0.0, smv=0.0,
                     timeout=0.5, blocking=False):
        d = d if d != None else self.defaultDepth
        h = h if h != None else self.curHeading
        goal = ControllerGoal(forward_setpoint=f, heading_setpoint=h,
                              sidemove_setpoint=sm, depth_setpoint=d,
                              forward_vel_setpoint=fv, sidemove_vel_setpoint=smv)
        self.motionClient.send_goal(goal)
        rospy.loginfo("Moving f:%lf, sm:%lf, h:%lf, d:%lf", f, sm, h, d)

        if blocking:
            self.motionClient.wait_for_result()
        else:
            self.motionClient.wait_for_result(timeout=rospy.Duration(timeout))
