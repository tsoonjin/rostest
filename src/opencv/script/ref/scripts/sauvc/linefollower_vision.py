import roslib; roslib.load_manifest('vision')
import rospy
import actionlib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server as DynServer

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
import vision.cfg.linefollowerConfig as Config

import cv2

import math
import numpy as np
import signal
from collections import deque

class LineFollower():
    testing = False
    thval = 30
    upperThresh = 70
    areaThresh = 3000
    upperAreaThresh = 100000
    screen = { 'width' : 640, 'height' : 480 }

    locomotionClient = actionlib.SimpleActionClient("LocomotionServer",
                                                    bbauv_msgs.msg.ControllerAction)

    curHeading = 0.0
    depth_setpoint = 0.6
    actionsHist = deque()

    def __init__(self):
        self.testing = rospy.get_param("~testing", False)
        #Handle signal
        signal.signal(signal.SIGINT, self.userQuit)

        self.isAborted = False
        self.isKilled = False
        self.cvbridge = CvBridge()
        self.rectData = {'detected':False}

        #Initialize Subscribers and Publishers
        self.image_topic = rospy.get_param('~image', '/bot_camera/camera/image_raw')
        self.registerSubscribers()
        #Publisher for testing output image
        self.outPub = rospy.Publisher("/Vision/image_filter", Image)

        # Set up dynamic reconfigure for linefollower
#         self.dyn_reconf_server = DynServer(Config, self.reconfigure)

        #Initialize mission planner communication server and client
        self.comServer = rospy.Service("/linefollower/mission_to_vision", mission_to_vision, self.handleSrv)

        if not self.testing:
            rospy.loginfo("Waiting for vision_to_mission server...")
            self.isAborted = True
            self.toMission = rospy.ServiceProxy("/linefollower/vision_to_mission",
                                                vision_to_mission)
            self.toMission.wait_for_service(timeout = 60)

        #Wait for locomotion server to start
        try:
            rospy.loginfo("Waiting for Locomotion Server...")
            self.locomotionClient.wait_for_server(timeout=rospy.Duration(5))
        except:
            rospy.loginfo("Locomotion Server timeout!")
            self.isKilled = True

        #Setting controller server
        if self.testing:
            setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
            setServer(forward=True, sidemove=True, heading=True, depth=True, pitch=True, roll=False,
                      topside=False, navigation=False)


    def userQuit(self, signal, frame):
        self.isAborted = True
        self.isKilled = True

    def reconfigure(self, config, level):
#         rospy.loginfo("Got dynamic reconfigure params")
#         self.areaThresh = config['area_thresh']
#         self.upperThresh = config['upper_thresh']
#
        return config

    def registerSubscribers(self):
        #Subscribe to camera
        rospy.loginfo(self.image_topic)
        self.imgSub = rospy.Subscriber(self.image_topic,
                                       Image,
                                       self.cameraCallback)
        #Subscribe to compass
        self.comSub = rospy.Subscriber("/euler",
                                        compass_data,
                                        self.compassCallback)

    def unregisterSubscribers(self):
        self.imgSub.unregister()
#         self.comSub.unregister()
        self.rectData['detected'] = False

    #Handle communication service with mission planner
    def handleSrv(self, req):
        if req.start_request:
            self.isAborted = False
            self.depth_setpoint = req.start_ctrl.depth_setpoint
        elif req.abort_request:
            rospy.loginfo("Got abort request")
            self.isAborted = True

        lastHeading = [self.curHeading, self.curHeading]
        length = len(self.actionsHist)
        if length > 1:
            lastHeading = self.actionsHist[-2]
        elif length > 0:
            lastHeading = self.actionsHist[-1]

        return mission_to_visionResponse(start_response=True, abort_response=False,
                                         data=controller(heading_setpoint=lastHeading[1]))

    def stopRobot(self):
        self.sendMovement(f=0, sm=0)

    #ROS callback functions
    def cameraCallback(self, image):
        try:
            cvImg = self.cvbridge.imgmsg_to_cv2(image, image.encoding)
        except CvBridgeError as e:
            rospy.logerr(e)

        self.detectBlackLine(cvImg)

    def compassCallback(self, data):
        self.curHeading = data.yaw

    #Utility function to send movements through locomotion server
    def sendMovement(self, f=0.0, h=None, sm=0.0, d=None, recordAction=True):
        d = d if d else self.depth_setpoint
        h = h if h else self.curHeading
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=f, heading_setpoint=h,
                                             sidemove_setpoint=sm, depth_setpoint=d)

        # Record actions to revert if necessary
        if recordAction:
            if len(self.actionsHist) > 10:
                self.actionsHist.popleft()
            self.actionsHist.append([f, h, sm, d])

        rospy.loginfo("Moving f:{}, h:{}, sm:{}, d:{}".format(f, h, sm, d))
        self.locomotionClient.send_goal(goal)
        self.locomotionClient.wait_for_result(rospy.Duration(0.5))

    def revertMovement(self):
        if len(self.actionsHist) == 0:
            return False

        rospy.loginfo("Reverting...")
        movements = self.actionsHist.popleft()
        # Reverse the direction of forward and sidemove
        movements[0] = -movements[0]
        movements[2] = -movements[2]
        self.sendMovement(*movements, recordAction=False)
        return True

    def abortMission(self):
        #Notify mission planner service
        if not self.testing:
            self.toMission(fail_request=True, task_complete_request=False)
        self.isAborted = True
        self.isKilled = True
        self.stopRobot()

    def taskComplete(self):
        if not self.testing:
            self.toMission(task_complete_request=True)
        self.stopRobot()
        self.isAborted = True
        self.isKilled = True

    # Main filters chain
    def detectBlackLine(self, img):
        grayImg = cv2.cvtColor(img, cv2.cv.CV_BGR2GRAY)
        grayImg = cv2.resize(grayImg, dsize=(self.screen['width'], self.screen['height']))
        grayImg = cv2.GaussianBlur(grayImg, ksize=(7, 7), sigmaX=0)

        # Calculate adaptive threshold value
        mean = cv2.mean(grayImg)[0]
        lowest = cv2.minMaxLoc(grayImg)[0]
        self.thval = min((mean + lowest) / 3.99, self.upperThresh)
        rospy.logdebug(self.thval)

        #Thresholding and noise removal
        grayImg = cv2.threshold(grayImg, self.thval, 255, cv2.THRESH_BINARY_INV)[1]

        #erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        openEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        #grayImg = cv2.erode(grayImg, erodeEl)
        grayImg = cv2.dilate(grayImg, dilateEl)
        grayImg = cv2.morphologyEx(grayImg, cv2.MORPH_OPEN, openEl)

        out = cv2.cvtColor(grayImg, cv2.cv.CV_GRAY2BGR)

        # Find centroid and bounding box
        pImg = grayImg.copy()
        contours, hierachy = cv2.findContours(pImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        maxArea = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.areaThresh and area < self.upperAreaThresh and area > maxArea:
                #Find the center using moments
                mu = cv2.moments(contour, False)
                centroidx = mu['m10'] / mu['m00']
                centroidy = mu['m01'] / mu['m00']
                maxArea = area

                self.rectData['centroid'] = (centroidx, centroidy)
                self.rectData['rect'] = cv2.minAreaRect(contour)

        if maxArea > 0:
            self.rectData['detected'] = True
            points = np.array(cv2.cv.BoxPoints(self.rectData['rect']))

            #Find the blackline heading
            edge1 = points[1] - points[0]
            edge2 = points[2] - points[1]

            #Choose the vertical edge
            if cv2.norm(edge1) > cv2.norm(edge2):
                edge1[1] = edge1[1] if edge1[1] != 0.0 else math.copysign(0.01, edge1[1])
                self.rectData['angle'] = math.degrees(math.atan(edge1[0]/edge1[1]))
            else:
                edge2[1] = edge2[1] if edge2[1] != 0.0 else math.copysign(0.01, edge2[1])
                self.rectData['angle'] = math.degrees(math.atan(edge2[0]/edge2[1]))

            #Chose angle to turn if horizontal
            if self.rectData['angle'] == 90:
                if self.rectData['centroid'][0] > self.screen['width'] / 2:
                    self.rectData['angle'] = -90
            elif self.rectData['angle'] == -90:
                if self.rectData['centroid'][0] < self.screen['width'] / 2:
                    self.rectData['angle'] = 90

            #Testing
            centerx = int(self.rectData['centroid'][0])
            centery = int(self.rectData['centroid'][1])
            cv2.circle(out, (centerx, centery), 5, (0, 255, 0))

            for i in range(4):
                pt1 = (int(points[i][0]), int(points[i][1]))
                pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
                cv2.line(out, pt1, pt2, (0, 0, 255))

            cv2.putText(out, str(self.rectData['angle']), (30, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))
        else:
            self.rectData['detected'] = False

        try:
            self.outPub.publish(self.cvbridge.cv2_to_imgmsg(out, encoding="bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)
