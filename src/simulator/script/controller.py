#!/usr/bin/env python 

import rospy 
import actionlib 
import time
import math
import numpy as np
from bbauv_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from tf.transformations import *

GZ_TOPIC = "/gazebo/model_states"
ROBOT = 'pr2'
WORLD = 'world'
class Controller:

    def __init__(self):
        rospy.init_node("MasterMind")
        #Intrinsic Properties
        self.pos = None
        self.ori = None
        self.twist = None
        self.yaw = 0
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist)
        self.set_model_client = None
        self.subscribe()

        #Locomotion servers 
        self.motionClient = actionlib.SimpleActionClient("LocomotionServer", ControllerAction)
        try:
            rospy.loginfo("Waiting for Locomotion Server...")
            self.motionClient.wait_for_server(timeout=rospy.Duration(1))
        except:
            rospy.loginfo("Locomotion server timeout!")

        #Waiting for gazebo set model state
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
           rospy.loginfo('Successfully connected to set_model_state service')
           self.set_model_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState) 
        except rospy.ServiceException,  e:
            rospy.logerr(e)

    def subscribe(self):
        self.gazebo_sub = rospy.Subscriber(GZ_TOPIC, ModelStates, self.gazebo_sub)

    def gazebo_sub(self, data):
        self.pos = data.pose[0].position
        self.ori = data.pose[0].orientation
        self.twist = data.twist
        self.euler_from_quart()
        self.vel_pub.publish(linear=Point(x=2, y=0, z=0), angular=self.twist[0].angular)
        #self.move_model()
        #rospy.loginfo("X: %.2f, Y: %.2f, Depth: %.2f, Heading: %.2f"%(self.pos.x, self.pos.y, self.pos.z, self.ori.x))

    def move_model(self, x=0, y=0, depth=0, heading=0):
        pose = Pose(position=Point(x=self.pos.x+x, y=self.pos.y+y,z=self.pos.z+depth), orientation=self.ori)
        twist = Twist(angular=self.twist[0].angular, linear=self.twist[0].linear)
        model_state = ModelState()
        model_state.model_name = ROBOT
        model_state.reference_frame = WORLD
        model_state.pose = pose
        model_state.twist = twist
        resp = self.set_model_client(model_state)

    def sendMovement(self, f, sm, t, d, timeout):

        goal = ControllerGoal(forward_setpoint=f, heading_setpoint=t, sidemove_setpoint=sm, depth_setpoint=d)
        self.motionClient.send_goal(goal)
        self.motionClient.wait_for_result(timeout=rospy.Duration(timeout))

    #utility function 

    def euler_from_quart(self): #angle returned 0 to 180 anti-clockwise 
        quat = (self.ori.x, self.ori.y, self.ori.z, self.ori.w)
        (roll, pitch, yaw) = euler_from_quaternion(quat)
        self.yaw = np.rad2deg(yaw)
        
if __name__ == '__main__':
    control = Controller()
    control.sendMovement(1.0, 2.0, 3.0, 4.0, 5)
    rospy.spin()
