#!/usr/bin/env python
import rospy
import os
import roslib
from bbauv_msgs.msg import * 
from bbauv_msgs.srv import * 
from sensor_msgs.msg import Image
import cv2
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError
import math
import time
import static
import signal
from config import Config
from utils.utils import Utils

'''
TASK: POLE DANCE
HORIZONTAL LENGTH: 1.8M 
VERTICAL LENGTH: 0.9M
VEHICLE WIDTH: 0.6M
'''

#GLOBAL
useImg = False
isTest = False

conn = Config()
class Img: 
    def __init__(self, img ,con, strr):
        img = cv2.resize(img, (640,480))
        self.hsv = static.toHsv(img)
        self.enhanced = static.powerUp(self.hsv)
        self.enhanced_bgr = cv2.cvtColor(self.enhanced, cv2.COLOR_HSV2BGR)
        if strr == "red":
            self.mask = static.inRange(self.hsv, con.red_min, con.red_max)
        elif strr == "green":
            self.mask = static.inRange(self.hsv, con.green_min, con.green_max)
        self.mask_bgr = cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)
        if strr == "red":
            self.cnt = static.findContour(self.mask)
        elif strr == "green":
            self.cnt = static.findContourGreen(self.mask)
        self.area= 0
        self.box = None
        self.mom = None
        self.centroid = None
        self.angle = None
        self.deltaX = None
        self.deltaY = None
        self.detected = False
        self.corner = None
        self.length = None
        self.width = None
        self.ends = None
        self.isSide = False
        self.isFrontal = False
        self.incline = None
        if self.cnt is not 0:
            self.area= cv2.contourArea(self.cnt) 
            self.box = static.getBox(self.cnt) 
            self.mom = cv2.moments(self.cnt) 
            self.centroid = static.getCentroid(self.mom, self.mask)
            angDir = static.getAngle(self.cnt)
            self.angle = int(angDir[0])
            self.incline = angDir[1]
            self.deltaX = (self.centroid[0] - con.center_w)/640.0
            self.deltaY = (self.centroid[1] - con.center_h)/480.0
            self.corner = static.getCorner(self.box)
            self.length = ((self.corner[0])[1] - (self.corner[2])[1])
            self.width = ((self.corner[1])[0] - (self.corner[0])[0])
        if self.ends:
            self.isFrontal = self.ends[0][1] < self.centroid[0] < self.ends[1][0] 
           


def isSide(img):
    while True:
        if img.ends:
            centroid_x = img.centroid[0]
            centroid_y = img.centroid[1]
            dY1 = abs(centroid_y - (img.ends[0])[1])
            dX1 = abs(centroid_x - (img.ends[0])[0])
            dY2 = abs(centroid_y - (img.ends[1])[1])
            dX2 = abs(centroid_x - (img.ends[1])[0])
            return (dY1/dX1) < 1 or (dY2/dX2) < 1 
        else:
            continue

def drawEnds(img, draw):
    x = [(i[0])[0] for i in img.cnt]
    y = [(i[0])[1] for i in img.cnt]
    top_left = (min(x), max(y))
    top_right = (max(x), max(y))
    bot_left = (min(x), min(y))
    bot_right = (max(x), min(y))
    cv2.circle(draw, (bot_left[0], bot_left[1]), 8, (255, 144, 30), -1)
    cv2.circle(draw, (top_right[0], top_right[1]), 8, (255,144,30), -1)
    return (top_left, top_right, bot_left, bot_right)

def drawCentroid(img, draw):
    centroid_x = img.centroid[0]
    centroid_y = img.centroid[1]
    cv2.circle(draw, (centroid_x, centroid_y), 6, (0,0,255), -1)

def drawBoundingGreen(img, draw):
    cv2.drawContours(draw, [img.cnt], -1, (0,255,0), 2)

def drawBounding(img, draw):
    cv2.drawContours(draw, [img.box], -1, (0,0,255), 2)

def drawCenter(draw):
    global conn
    cv2.rectangle(draw, (conn.center_w - 5, conn.center_h + 5), (conn.center_w + 5, conn.center_h - 5), (0,255,0), 1)

def drawCrossHair(draw):
    global conn
    cv2.line(draw, (conn.center_w - 5, conn.center_h), (conn.center_w + 5, conn.center_h), (127,0,255))   
    cv2.line(draw, (conn.center_w, conn.center_h + 5), (conn.center_w, conn.center_h - 5) , (127,0,255))
    cv2.circle(draw, (conn.center_w, conn.center_h), 6, (127,0,255))


class PoleVision():

    def __init__(self):
        self.bridge = CvBridge()
        self.con = Config()

        self.center = {'x': 160, 'y': 120}
        self.corners = [] #top-left, top-right, bot-left, bot-right
        signal.signal(signal.SIGINT, self.signalHandler)
        if isTest:
            self.img_pub = rospy.Publisher("/Vision/image_filter_jin_bag", Image)
            self.img_pub2 = rospy.Publisher("/Vision/image_filter_enhanced_bag", Image)
            self.img_sub = rospy.Subscriber("/front_camera/camera/image_raw_jin", Image, self.camCallback)


    def gotFrame(self, img):
        red_img = Img(img, self.con, "red")
        copy_img = img.copy()
        green_img = Img(copy_img, self.con, "green")
        total_img = cv2.bitwise_or(red_img.mask_bgr, green_img.mask_bgr)
        if red_img.cnt is not 0:
            drawCentroid(red_img, red_img.mask_bgr)
            drawCentroid(red_img, total_img)
            drawBounding(red_img, red_img.mask_bgr)
            drawBounding(red_img, total_img)
            red_img.detected = True
         
        if green_img.cnt is not 0:
            #drawCentroid(green_img, green_img.mask_bgr)
            green_img.ends = drawEnds(green_img, green_img.mask_bgr)
            drawEnds(green_img, total_img)
            drawBoundingGreen(green_img, green_img.mask_bgr)
            drawBoundingGreen(green_img, total_img)
            drawCentroid(green_img, green_img.mask_bgr)
            drawCentroid(green_img, total_img)
            green_img.detected = True
            green_img.isSide = isSide(green_img)
            gradient = "None"
            if green_img.ends:
                centroid_x = green_img.centroid[0]
                centroid_y = green_img.centroid[1]
                dY1 = abs(centroid_y - (green_img.ends[0])[1])
                dX1 = abs(centroid_x - (green_img.ends[0])[0])
                gradient = dY1/float(dX1)
            cv2.putText(green_img.mask_bgr, ("Gradient: "+str(gradient)), (440, 90), cv2.FONT_HERSHEY_PLAIN, 1, (0,252,124))
            cv2.putText(total_img, ("Gradient: "+str(gradient)), (440, 90), cv2.FONT_HERSHEY_PLAIN, 1, (0,252,124))

        drawCrossHair(red_img.mask_bgr) 
        drawCrossHair(total_img) 
        self.debugText(red_img, red_img.mask_bgr)
        self.debugText(red_img, total_img)
        drawCrossHair(green_img.mask_bgr) 
        self.debugTextGreen(green_img, green_img.mask_bgr)
        self.debugTextGreen(green_img, total_img)
        cv2.putText(total_img, "musJin", (20,460),  cv2.FONT_HERSHEY_DUPLEX, 1, (211,0,148))
        return red_img, green_img, total_img
       

    def debugText(self, img_data, draw):
        cv2.putText(draw, ("Area: "+str(img_data.area)), (440, 400), cv2.FONT_HERSHEY_PLAIN, 1, (0,140,255))
        cv2.putText(draw, ("Angle: "+str(img_data.angle)), (440, 415), cv2.FONT_HERSHEY_PLAIN, 1, (0,140,255))
        cv2.putText(draw, ("DeltaX: "+str(img_data.deltaX)), (440, 430), cv2.FONT_HERSHEY_PLAIN, 1, (0,140,255))
        cv2.putText(draw, ("DeltaY: "+str(img_data.deltaY)), (440, 445), cv2.FONT_HERSHEY_PLAIN, 1, (0,140,255))
        cv2.putText(draw, ("Detected: "+str(img_data.detected)), (440, 460), cv2.FONT_HERSHEY_PLAIN, 1, (0,140,255))

    def debugTextGreen(self, img_data, draw):

        if not img_data.ends:
            l_end =" (None, None)"
            r_end =" (None, None)"
        else:
            l_end = img_data.ends[0]
            r_end = img_data.ends[1]
        cv2.putText(draw, ("Area: "+str(img_data.area)), (440, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0,252,124))
        cv2.putText(draw, ("Detected: "+str(img_data.detected)), (440, 35), cv2.FONT_HERSHEY_PLAIN, 1, (0,252,124))
        cv2.putText(draw, ("L_end: "+str(l_end)), (440, 60), cv2.FONT_HERSHEY_PLAIN, 1, (0,252,124))
        cv2.putText(draw, ("R_end: "+str(r_end)), (440, 75), cv2.FONT_HERSHEY_PLAIN, 1, (0,252,124))
    def camCallback(self, img):
        rospy.loginfo("Solo")
        img = Utils.rosimg2cv(img) 
        red_img = Img(img, conn)
        if(1000 < red_img.area < 1500):
            red_img.drawBounding(red_img.mask_bgr)
            red_img.drawCentroid(red_img.mask_bgr)
        drawCenter(red_img.mask_bgr)
        self.img_pub.publish(Utils.cv2rosimg(red_img.mask_bgr))
        self.img_pub2.publish(Utils.cv2rosimg(red_img.enhanced_bgr))

    def imgCallback(self, data):
        cv_img = ros2cv(data)
        red_img = Img(cv_img, self.con)
        red_img.drawCentroid(red_img.mask_bgr)
        red_img.drawBounding(red_img.mask_bgr)
        #green_img = Img(cv_img.copy())
        output_mask = cv2ros(red_img.mask_bgr)
        output_enhance = cv2ros(red_img.enhanced)
        self.img_pub.publish(output_mask)
        self.img_pub2.publish(output_enhance)
        time.sleep(0.05)

    def signalHandler(self, signum, frame):
        rospy.signal_shutdown("Killed")
            


def main():
    global conn
    rospy.init_node("round_vision")
    rospy.loginfo("Init")
    if useImg:
        dir = os.path.dirname(os.path.abspath(__file__))
        path = dir + "/test.jpg"
        test_img = cv2.imread(path, 1)
        cv2.namedWindow("mask")
        cv2.namedWindow("ori")
        red_img = Img(test_img, conn)
        red_img.drawBounding(red_img.mask_bgr)
        red_img.drawCentroid(red_img.mask_bgr)
        drawCenter(red_img.mask_bgr)
        cv2.imshow("mask", red_img.mask_bgr)
        cv2.imshow("ori", test_img)
        k = cv2.waitKey(0) 
        if k == 27:
            cv2.destroyAllWindows()
    else:
        rospy.loginfo("Subscribed")
        vis = PoleVision()
        rospy.spin()
        
if __name__ == "__main__":
    main()
