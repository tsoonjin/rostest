#/usr/bin/env/python 

'''
Vision filter chain for the moving of pegs Task
'''

import math
import numpy as np
import cv2

from utils.utils import Utils
from front_commons.frontCommsVision import FrontCommsVision as vision
import rospy

class PegsVision:
    screen = {'width': 640, 'height': 480}
    
    #Vision parameters - pegs are either red or white

    redParams = {'lo1': (113, 0, 0), 'hi1': (136, 255, 255),
                 'lo2': (0, 0, 0), 'hi2': (27, 255, 255),
                 'lo3': (150, 223, 0), 'hi3': (255, 255, 255),  # Red white balance values 
                 'dilate': (15,15), 'erode': (5,5), 'open': (3,3)}
    
    blueParams = {'lo': (97, 0, 0), 'hi': (139, 255, 255),
                  'dilate': (13,13), 'erode': (5,5), 'open': (5,5)}
    
    # For hough circle params
    circleParams = {'minRadius': 0, 'maxRadius': 200}
    houghParams = (350, 13)

    minContourArea = 100
    
    prevCentroid = (-1, -1)
    prevArea = 0
    
    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms
        
    def gotFrame(self, img):
        #Set up parameters
        allCentroidList = []
        allAreaList = []
        self.comms.foundSomething = False 
        
        outImg = None
        
        #Preprocessing 
        img = cv2.resize(img, (640, 480))
        
        # White balance
        img = vision.whiteBal(img)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = np.array(hsvImg, dtype=np.uint8)
                
        # Blur image
        gauss = cv2.GaussianBlur(hsvImg, ksize=(5,5), sigmaX=9)
        sum = cv2.addWeighted(hsvImg, 1.5, gauss, -0.6, 0)
        enhancedImg = cv2.medianBlur(sum, 3)
        
        return cv2.cvtColor(enhancedImg, cv2.COLOR_HSV2BGR)
        
        # Threshold red 
        params = self.redParams

        # Perform thresholding
#         binImg1 = cv2.inRange(hsvImg, params['lo1'], params['hi1'])
#         binImg2 = cv2.inRange(hsvImg, params['lo2'], params['hi2'])
#         binImg = cv2.bitwise_or(binImg1, binImg2)
#         binImg = self.erodeAndDilateImg(binImg, params)

        # Perform thresholding
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        mask = cv2.inRange(enhancedImg, self.redParams['lo3'], self.redParams['hi3'])
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSe, kern)
        kern2 = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        threshImg = cv2.dilate(mask, kern2, iterations=3)

        return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Find contours 
        scratchImgCol = cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)    # To overlay with centroids
        
        scratchImg = binImg.copy()  # For contours to mess up
        contours, hierachy = cv2.findContours(scratchImg, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        
        if len(contours) == 0:
            return scratchImgCol
        
        contours = filter(lambda c: cv2.contourArea(c) > self.minContourArea, contours)
        
        sorted(contours, key=cv2.contourArea, reverse=True) # Sort by largest contour 

        if self.comms.centering:
            self.comms.foundSomething = True
            # Find largest contour
            largestContour = contours[0]
            mu = cv2.moments(largestContour)
            muArea = mu['m00']
            self.comms.centroidToPick = (int(mu['m10']/muArea), int(mu['m01']/muArea))
            self.comms.areaRect = cv2.minAreaRect(largestContour)

            rospy.loginfo("Area of centroid:{}".format(self.comms.areaRect))
            
            # Draw new centroid
            cv2.circle(scratchImgCol, self.comms.centroidToPick, 3, (0, 255, 255), 2)
            # How far the centroid is off the screen center
            
            self.comms.deltaX = float((vision.screen['width']/2 - self.comms.centroidToPick[0])*1.0/vision.screen['width'])                                                                                                                                          
            cv2.putText(scratchImgCol, str(self.comms.deltaX), (30,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
            self.comms.deltaY = float((self.comms.centroidToPick[1] - vision.screen['height']/2)*1.0/
                                      vision.screen['height'])
            cv2.putText(scratchImgCol, "Y  " + str(self.comms.deltaY), (30,60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))
            
            cv2.putText(scratchImgCol, "Area " + str(self.comms.areaRect), (30,85),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))
            
            return scratchImgCol

        # When not centering find a circle
        # Find Hough circles        
        circles = cv2.HoughCircles(binImg, cv2.cv.CV_HOUGH_GRADIENT, 1,
                                   minDist=1, param1=self.houghParams[0], 
                                   param2=self.houghParams[1],
                                   minRadius = self.circleParams['minRadius'],
                                   maxRadius = self.circleParams['maxRadius'])

        # Check if centroid of contour is inside a circle
        for contour in contours:
            mu = cv2.moments(contour)
            muArea = mu['m00']
            centroid = (mu['m10']/muArea, mu['m01']/muArea)
            
            if circles is None:
                self.comms.foundSomething = False 
                return scratchImgCol
            
            for circle in circles[0,:,:]:
                circleCentroid = (circle[0], circle[1])
                if abs((Utils.distBetweenPoints(centroid, circleCentroid))) < circle[2]:
                    self.comms.foundSomething = True
                    # Find new centroid by averaging the centroid and the circle centroid
                    newCentroid = (int(centroid[0]+circleCentroid[0])/2, 
                                   int(centroid[1]+circleCentroid[1])/2)
                    allCentroidList.append(newCentroid)
                    allAreaList.append(cv2.contourArea(contour))
                        
                    # Draw Circles
                    cv2.circle(scratchImgCol, newCentroid, circle[2], (255, 255, 0), 2)
                    cv2.circle(scratchImgCol, newCentroid, 2, (255, 0, 255), 3)
                        
                    break       
                     
        # Centroid resetted 
        if self.comms.centroidToPick == None:
            if not len(allCentroidList) == 0:                
                # Pick the nth centroid to hit 
                self.comms.centroidToPick = allCentroidList[self.comms.count]
                self.comms.areaRect = allAreaList[self.comms.count]
                    
                self.prevCentroid = self.comms.centroidToPick
                self.prevArea = self.comms.areaRect
                
        else:
            if not len(allCentroidList) == 0:            
                # Compare with the previous centroid and pick the one nearest
                for centroid in allCentroidList:
                    distDiff = []
                    distDiff.append(Utils.distBetweenPoints(
                                        self.previousCentroid, centroid))
                minIndex = distDiff.index(min(distDiff))
                self.comms.centroidToPick = allCentroidList[minIndex]
                self.comms.areaRect = allAreaList[minIndex]
                
                self.prevCentroid = self.comms.centroidToPick
                self.prevArea = self.comms.areaRect
                
            else:
                # Use back the previous
                self.comms.centroidToPick = self.previousCentroid
                self.comms.areaRect = self.previousArea
                 
            rospy.loginfo("Area: {}".format(self.comms.areaRect))
                 
            # Draw new centroid
            cv2.circle(scratchImgCol, self.comms.centroidToPick, 3, (0, 255, 255), 2)
                 
            # How far the centroid is off the screen center
            self.comms.deltaX = float((vision.screen['width']/2 - self.comms.centroidToPick[0])*1.0/vision.screen['width'])                                                                                                                                          
            cv2.putText(scratchImgCol, str(self.comms.deltaX), (30,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
              
        return scratchImgCol
         
    def erodeAndDilateImg(self, image, params):      
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, params['erode'])
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, params['dilate'])
        closeEl = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, params['open'])

        image = cv2.erode(image, erodeEl)
        image = cv2.dilate(image, dilateEl)
        image = cv2.morphologyEx(image, cv2.MORPH_OPEN, closeEl)  
        
        return image
    
    def updateParams(self):
        self.redParams['lo1'] = self.comms.params['loThreshold']
        self.redParams['hi1'] = self.comms.params['hiThreshold']
        self.houghParams = self.comms.params['houghParams']
        self.minContourArea = self.comms.params['minContourArea']
        
def main():
    cv2.namedWindow("Peg Test")
    inImg = cv2.imread("pegs/pegs3.png")
    from comms import Comms
    detector = PegsVision(comms = Comms())
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Peg Test", outImg)
    cv2.waitKey()
    
    