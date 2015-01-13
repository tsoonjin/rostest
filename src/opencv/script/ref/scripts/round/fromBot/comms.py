#/usr/bin/env/python 
'''
For communication with Robot 
'''
import rospy
import roslib
import signal
from vision import PoleVision
from front_commons.frontComms import FrontComms
from bbauv_msgs.srv import *
from bbauv_msgs.msg import *
from sensor_msgs.msg import Image
from utils.utils import Utils
import time
import cv2
#from dynamic_reconfigure.server import Server
#from round.cfg import roundConfig

from config import Config

#GLOBAL

class Comms(FrontComms):

    # Vision boolean
    foundSomething = False 
    
    
    areaRect = None

    #vis = vision.PoleVision()
    
    def __init__(self):
        FrontComms.__init__(self, PoleVision())
        signal.signal(signal.SIGINT, self.quit)
        signal.signal(signal.SIGTERM, self.quit)
        self.con = Config()
        #self.dynServer = Server(self.con, self.reconfigure)
        self.curHeading = None
        self.currDepth = None
        self.setDepth = 2.0
        self.newDepth = None
        self.initTime = 0
        self.isCheck = False
        self.isDone = False 
        self.strike_distance = 0.7
        self.red_img = None
        self.green_img = None
        self.total_img = None
        self.red_img = None
        self.green_img = None
        self.isAborted = True
        self.isStart = False
        if self.isAlone:
            self.isStart = True
            self.isAborted = False

        if not self.isAlone:
            self.server = rospy.Service("/pole/mission_to_vision", mission_to_vision, self.handle_srv)
            rospy.loginfo("Waiting for mission, mission_to_vision service running")
            self.send2Mission = rospy.ServiceProxy("/pole/vision_to_mission",vision_to_mission)
            self.send2Mission.wait_for_service(timeout=50)
        
        
        
    # Handle mission services
    def handle_srv(self, req):
        rospy.loginfo("Pole Service handled")
        
        if req.start_request:
            rospy.loginfo("Pole starting") 
            self.isStart = True
            self.isAborted = False
            self.setDepth = req.start_ctrl.depth_setpoint
        
        if req.abort_request:
            rospy.loginfo("Pole abort received")
            self.isAborted=True
            self.isStart = False
            self.unregisterCurr()
            
        return mission_to_visionResponse(self.isStart, self.isAborted)
    
    def registerCurr(self):
        if not self.onBot:
            self.camSub = rospy.Subscriber(self.con.camTopic + "_jin", Image, self.camera_callback)
            self.outPub = rospy.Publisher(self.con.pubTopic + "_bag", Image)
            self.outPub2 = rospy.Publisher(self.con.enhanceTopic + "_bag", Image)
            self.outPub3 = rospy.Publisher(self.con.pubTopic2 + "_bag", Image)
            self.outPub4 = rospy.Publisher(self.con.pubTopic3 + "_bag", Image)
        else:
            self.camSub = rospy.Subscriber(self.con.camTopic, Image, self.camera_callback)
            self.outPub = rospy.Publisher(self.con.pubTopic, Image)
            self.outPub2 = rospy.Publisher(self.con.enhanceTopic, Image)
            self.outPub3 = rospy.Publisher(self.con.pubTopic2, Image)
            self.outPub4 = rospy.Publisher(self.con.pubTopic3 , Image)
        self.compassSub = rospy.Subscriber(self.con.compassTopic,
                                               compass_data,
                                               self.compassCallback)
        self.depthSub = rospy.Subscriber(self.con.depthTopic, depth, self.depthCallback)

    def quit(self, signum, frame):
        self.isAborted = True
        self.isStart = False
        rospy.signal_shutdown("Killed")

    def unregisterCurr(self):
        if self.camSub is not None:
            self.camSub.unregister()
        if self.compassSub is not None:
            self.compassSub.unregister()
        if self.depthSub is not None:
            self.depthSub.unregister()
        self.canPublish = False 

    def sendMovement(self,forward=0.0, sidemove=0.0, turn=None, depth=None, absolute=False, wait=True, timeout=0.2):

        depth = depth if depth else self.setDepth
        if turn is None:
            turn = self.curHeading
            goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
        else:
            if absolute:
                goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
            else:
                turn = (turn+self.curHeading)%360 
                goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
        rospy.loginfo("F: " + str(forward) + ", T: " + str(turn) + " S: " + str(sidemove) + ", D: " + str(depth))
        self.motionClient.send_goal(goal)
        if wait:
            self.motionClient.wait_for_result()
        else:
            self.motionClient.wait_for_result(timeout=rospy.Duration(timeout))
           
    def getAvg(self, img):
        container = []
        for i in xrange(10):
            container.append(img)
            time.sleep(0.5)
        return container

    def depthCallback(self, data):  
        self.currDepth = data.depth

    def compassCallback(self, data):
        if not self.gotHeading:
            self.curHeading = data.yaw

    def camera_callback(self, rosImg):
        self.red_img, self.green_img, self.total_img = self.visionFilter.gotFrame(Utils.rosimg2cv(rosImg))
        if rosImg is not None and self.canPublish:
            #self.outPub.publish(Utils.cv2rosimg(self.red_img.mask_bgr))
            self.outPub2.publish(Utils.cv2rosimg(self.red_img.enhanced_bgr))
            #self.outPub3.publish(Utils.cv2rosimg(self.green_img.mask_bgr))
            self.outPub4.publish(Utils.cv2rosimg(self.total_img))
        time.sleep(0.01)


    def reconfigure(self, config, level):
        #rospy.loginfo("Receive dynamic reconfigure request")
        self.params = {'hsvLoThresh1' : (config.loH, config.loS, config.loV),
                       'hsvHiThresh1' : (config.hiH, config.hiS, config.hiV),
                       'minContourArea' : config.minArea}
        self.visionFilter.updateParams()
        #rospy.loginfo("Params: {}".format(str(self.params)))
        return config

def main():
    pass
    
    
