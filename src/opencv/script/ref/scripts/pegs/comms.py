#/usr/bin/env/python 

'''
For communication with Robot 
'''

import rospy
from front_commons.frontComms import FrontComms
from vision import PegsVision

from dynamic_reconfigure.server import Server as DynServer
from utils.config import pegsConfig as Config

from bbauv_msgs.msg._manipulator import manipulator
from bbauv_msgs.msg import controller, sonarData
from bbauv_msgs.srv import mission_to_visionResponse, \
        mission_to_vision, vision_to_mission

class Comms(FrontComms):
    
    isTesting = False
    isKilled = False 
    isAborted = True
    isStart = False
    
    # Vision parameters     
    findRedPeg = True   #Either find red or find blue circle 
    foundSomething = False 
    
    count = 0       # Move up to 4 pegs 
    
    centroidToPick = None
    deltaX = 0
    areaRect = 0
    centering = False 
    
    def __init__(self):
        FrontComms.__init__(self, PegsVision(comms=self))
        self.defaultDepth = 2.0
        
        self.dynServer = DynServer(Config, self.reconfigure)
        
        # Initialise mission planner 
        if not self.isAlone:
            self.comServer = rospy.Service("/pegs/mission_to_vision", 
                                           mission_to_vision,
                                           self.handle_srv)
            rospy.loginfo("Waiting for mission planner")
            self.toMission = rospy.ServiceProxy("/pegs/vision_to_mission",
                                                vision_to_mission)
            self.toMission.wait_for_service()
        
    # Handle mission services
    def handle_srv(self, req):
        global isStart
        global isAborted
        global locomotionGoal
                
        rospy.loginfo("Pegs Service handled")
        
        if req.start_request:
            rospy.loginfo("Pegs starting")
            self.isStart = True
            self.isAborted = False
            self.canPublish = True

            self.defaultDepth = req.start_ctrl.depth_setpoint
            self.inputHeading = req.start_ctrl.heading_setpoint
            self.curHeading = self.inputHeading

            self.registerMission()

            rospy.loginfo("Received heading: {}".format(self.inputHeading))
            rospy.loginfo("Received depth: {}".format(self.defaultDepth))

            return mission_to_visionResponse(start_response=True,
                                             abort_response=False,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))
        
        elif req.abort_request:
            rospy.loginfo("Pegs abort received")
            self.sendMovement(forward=0.0, sidemove=0.0)
            self.isAborted=True
            self.isStart = False
            self.canPublsih = False

            self.unregisterMission()
            
            return mission_to_visionResponse(start_response=False,
                                             abort_response=True,
                                             data=controller(heading_setpoint=self.curHeading))

    def grabRedPeg(self):
        maniPub = rospy.Publisher("/manipulators", manipulator)
        maniPub.publish(0 | 4)
        rospy.sleep(rospy.Duration(0.3))
        
    def putPeg(self):
        maniPub = rospy.Publisher("/manipulators", manipulator)
        maniPub.publish(1 & 4)
        rospy.sleep(rospy.Duration(0.3))

    def reconfigure(self, config, level):
        rospy.loginfo("Received dynamic reconfigure request")
        self.params = {'loThreshold': (config.loH, config.loS, config.loV),
                       'hiThreshold': (config.hiH, config.hiS, config.hiV),
                       'houghParams': (config.Hough1, config.Hough2),
                       'minContourArea': config.minContourArea }
        
        self.visionFilter.updateParams()
        return config
    
    def registerSonar(self):
        self.sonarBearing = None
        self.sonarDist = None 
        self.sonarSub = rospy.Subscriber("/sonarData", sonarData, self.sonarDataCallback)
    
    def sonarDataCallback(self, data):
        self.sonarBearing = data.bearing
        self.sonarDist = data.range
        
    def unregisterSonar(self):
        self.sonarSub.unregister()    

def main():
    testCom = Comms()
        
