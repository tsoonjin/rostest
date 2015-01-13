#!/usr/bin/env python
from PyQt4 import QtGui, QtCore
from PyQt4 import uic
from smart import Ui_MainWindow
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from nav_msgs.msg import Odometry
import sys
import rospy
import roslib
import actionlib
import time
import signal
import random
import os

class AcousticPanel(QtGui.QMainWindow):

    def __init__(self):
        super(AcousticPanel, self).__init__()
        path = os.path.dirname(os.path.abspath(__file__)) 
        self.ui = uic.loadUi(path + "/untitled.ui", self)

        #AUV info
        self.depth = 0
        self.heading = 0
        self.doa = 0
        self.elevation = 0
        self.iteration = 0
        self.isTest = False

        #setup timer
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.timing)
        self.timer.start(1000)


        #Setup subscribers
        self.compass_sub = rospy.Subscriber("/euler", compass_data, self.compass_callback)
        self.depth_sub = rospy.Subscriber("/depth", depth, self.depth_callback)
        self.acoustic_sub = rospy.Subscriber("/acoustic/pingu", pingu, self.acoustic_callback)


        rospy.loginfo("Setting up the boxes")
        self.ui.pushButton.clicked.connect(self.buttonHandlerDistance)
        self.ui.pushButton_2.clicked.connect(self.buttonHandlerTurn)


    def timing(self):
        self.ui.textBrowser_2.setText("%.2f" % self.doa)
        self.ui.textBrowser_3.setText("%.2f" % self.elevation)
        self.ui.textBrowser_5.setText("%.2f" % self.heading)
        self.ui.textBrowser_6.setText("%.2f" % self.depth)

    def buttonHandlerDistance(self):
        distance = float(self.ui.lineEdit_12.text())
        rospy.loginfo("Moving robot")
        sendMovement(forward=distance)

    def buttonHandlerTurn(self):
        rospy.loginfo("Turning robot")
        sendMovement(forward=0.0, turn=self.doa)

    def initMovement(self):
        #Handle locomotion 
        try:
            self.locomotionClient = actionlib.SimpleActionClient('LocomotionServer',ControllerAction) 
            rospy.loginfo("Waiting for action server")
            self.locomotionClient.wait_for_server(timeout=rospy.Duration(20.0))
        except rospy.ServiceException:
            rospy.logerr("Error running Locmotion Client")
    #Initialise Controller Service:
        try:
            self.controllerSettings = rospy.ServiceProxy("/set_controller_srv", set_controller)
            rospy.loginfo("Waiting for controller server")
            self.controllerSettings.wait_for_service(timeout=10)
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to Controller")


    #Setting controller server
        setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
        setServer(forward=True, sidemove=True, heading=True, depth=False, pitch=True, roll=True, topside=False, navigation=False)

    def sendMovement(self,forward=0.0, sidemove=0.0, turn=None, depth=1.0, absolute=False, wait=True):
        if turn is None:
            turn = self.heading
            goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
        else:
            if absolute:
                goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
            else:
                turn = (turn+self.heading)%360 
                goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
        rospy.loginfo("Turn received: " + str(turn))
        self.locomotionClient.send_goal(goal)
        if wait:
            self.locomotionClient.wait_for_result()
        else:
            self.locomotionClient.wait_for_result(timeout=rospy.Duration(2.0))

    def compass_callback(self, data):
        self.heading = data.yaw

    def depth_callback(self, data):
        self.depth = data.depth
        rospy.loginfo("Depth")

    def acoustic_callback(self, data):
        self.doa = data.doa
        self.elevation = data.elevation



def cleanup(signum, frame):
    rospy.signal_shutdown("Terminated by user")

if __name__ == "__main__":
    rospy.init_node("acousticPanel")
    signal.signal(signal.SIGTERM, cleanup)
    app = QtGui.QApplication(sys.argv)
    window = AcousticPanel()
    #window.initMovement()
    window.show()
    sys.exit(app.exec_())
