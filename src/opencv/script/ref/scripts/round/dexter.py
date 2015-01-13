#!/usr/bin/env python
import rospy
import roslib 
import cv2
import numpy as np
from sensor_msgs.msg import Image
from utils.utils import Utils
import static
import time

class Dexter:
    
    def __init__(self):
       rospy.init_node("dexter")

       self.img_topic = rospy.get_param("~img", "/front_camera/camera/image_raw_jin") 


       self.img_sub = rospy.Subscriber(self.img_topic, Image, self.imgCallback)
       self.cvImg = None
       self.enhanceImg = None
       self.maskImg = None
       cv2.namedWindow("img", cv2.WINDOW_NORMAL)
       self.isWhite = 0
       self.isIllum = 0
       self.isCLAHE = 0
       self.lowH = 0
       self.highH = 180
       self.lowS = 0
       self.highS = 255
       self.lowV = 0
       self.highV = 255
       self.min = np.array([self.lowH,self.lowS, self.lowV], dtype='uint8')
       self.max = np.array([self.highH, self.highS, self.highV], dtype='uint8')

    def lowH_cb(self, lowH):
        self.lowH = lowH

    def highH_cb(self, highH):
        self.highH = highH

    def lowS_cb(self, lowS):
        self.lowS = lowS

    def highS_cb(self, highS):
        self.highS = highS

    def lowV_cb(self, lowV):
        self.lowV = lowV

    def highV_cb(self, highV):
        self.highV = highV

    def white_cb(self, white):
        self.isWhite = 1 if white == 1 else 0

    def illum_cb(self, illum):
        self.isIllum = 1 if illum == 1 else 0

    def clahe_cb(self, clahe):
        self.isCLAHE = 1 if clahe == 1 else 0

    def imgCallback(self,rosImg):
        self.cvImg = Utils.rosimg2cv(rosImg)
        self.cvImg = cv2.resize(np.array(self.cvImg, dtype=np.uint8), (160, 120))
        self.enhance(self.cvImg.copy())
        self.mask(self.cvImg.copy())
        
        
        
    def enhance(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img = np.array(img, dtype=np.uint8)
        self.enhanceImg = static.powerUp(img)
        if self.isWhite == 1:
            self.enhanceImg = static.whiteBal(self.enhanceImg)
        if self.isIllum == 1:
            self.enhanceImg = static.equalHist(self.enhanceImg)
        

    def mask(self, img):
        self.maskImg = static.inRange(img, self.min, self.max)
        
        
    def display(self):
        cv2.imshow("img", self.cvImg)
        self.createTrackbar("img")
        key = cv2.waitKey(0) 
        while key is not 27:
            enhanceImg = cv2.cvtColor(self.enhanceImg, cv2.COLOR_HSV2BGR)
            maskImg = cv2.cvtColor(self.maskImg, cv2.COLOR_GRAY2BGR)
            #final_img = np.hstack((self.cvImg, enhanceImg, maskImg))
            cv2.imshow("img", self.cvImg)
        cv2.destroyAllWindows()
        
        
def createTrackbar(self, window):
        cv2.createTrackbar("LowH", window, 0, 180, self.lowH_cb)
        cv2.createTrackbar("HighH", window, 0, 180, self.highH_cb)
        cv2.createTrackbar("LowS", window, 0, 255, self.lowS_cb)
        cv2.createTrackbar("HighS", window, 0, 255, self.highS_cb)
        cv2.createTrackbar("LowV", window, 0, 255, self.lowV_cb)
        cv2.createTrackbar("HighV", window, 0, 255, self.highV_cb)
        cv2.createTrackbar("White Balance", window, 0, 1, self.white_cb)
        cv2.createTrackbar("Illumination control", window, 0, 1, self.illum_cb)
        cv2.createTrackbar("CLAHE", window, 0, 1, self.clahe_cb)

def main():
    dexter = Dexter()
    dexter.display()

if __name__ == "__main__":
    dexter = Dexter()
    dexter.display()
