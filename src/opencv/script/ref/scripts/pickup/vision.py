import math
import numpy as np
import cv2

from utils.utils import Utils
from bot_common.vision import Vision


class PickupVision:
    SITE = 0
    SAMPLES = 1
    BOX = 2

    screen = {'width': 640, 'height': 480}

    # Vision parameters
    greenLoThresh = (35, 0, 0)
    greenHiThresh = (70, 255, 255)

    redLoThresh1 = (1, 0, 0)
    redHiThresh1 = (25, 255, 255)
    redLoThresh2 = (160, 0, 0)
    redHiThresh2 = (180, 255, 255)

    yellowLoThresh = (40, 0, 0)
    yellowHiThresh = (80, 255, 255)
    yellowLoThresh2 = (40, 0, 0)
    yellowHiThresh2 = (80, 255, 255)
    yellowLoThresh3 = (71, 170, 122)
    yellowHiThresh3 = (82, 255, 164)

    minContourArea = 5000
    maxContourArea = 50000
    minSiteArea = 10000

    minBoxArea = 700

    def __init__(self, comms=None, debugMode=True):
        self.comms = comms
        self.debugMode = debugMode

    def updateParams(self):
        self.greenLoThresh = self.comms.params['greenLoThresh']
        self.greenHiThresh = self.comms.params['greenHiThresh']
        self.redLoThresh1 = self.comms.params['redLoThresh1']
        self.redHiThresh1 = self.comms.params['redHiThresh1']
        self.redLoThresh2 = self.comms.params['redLoThresh2']
        self.redHiThresh2 = self.comms.params['redHiThresh2']
        self.yellowLoThresh = self.comms.params['yellowLoThresh']
        self.yellowHiThresh = self.comms.params['yellowHiThresh']
        self.yellowLoThresh2 = self.comms.params['yellowLoThresh2']
        self.yellowHiThresh2 = self.comms.params['yellowHiThresh2']

        self.minSiteArea = self.comms.params['minSiteArea']
        self.minContourArea = self.comms.params['minContourArea']
        self.maxContourArea = self.comms.params['maxContourArea']

    def morphology(self, img):
        # Closing up gaps and remove noise with morphological ops
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        img = cv2.erode(img, erodeEl)
        img = cv2.dilate(img, dilateEl)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, closeEl, iterations=5)

        return img

    def morphologyBox(self, img):
        # Closing up gaps and remove noise with morphological ops
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        img = cv2.erode(img, erodeEl)
        img = cv2.dilate(img, dilateEl)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, closeEl, iterations=3)

        return img

    def morphologyCheese(self, img):
        # Closing up gaps and remove noise with morphological ops
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
        #dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        closeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        img = cv2.erode(img, erodeEl)
        #img = cv2.dilate(img, dilateEl)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, closeEl, iterations=3)

        return img

    def morphologyRock(self, img):
        # Closing up gaps and remove noise with morphological ops
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
        #dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        closeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        img = cv2.erode(img, erodeEl)
        #img = cv2.dilate(img, dilateEl)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, closeEl, iterations=3)

        return img

    def findCenter(self, contour):
        moment = cv2.moments(contour, False)
        return (moment['m10']/moment['m00'],
                moment['m01']/moment['m00'])

    def findContourAndBound(self, img, bounded=True, upperbounded=False,
                            bound=0, upperbound=75000):
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        if bounded:
            contours = filter(lambda c: cv2.contourArea(c) > bound, contours)
        if upperbounded:
            contours = filter(lambda c: cv2.contourArea(c) < upperbound, contours)

        return contours

    def findSamples(self, category, binImg, samples, outImg):
        contours = self.findContourAndBound(binImg.copy(), bounded=True,
                                            upperbounded=True,
                                            bound=self.minContourArea,
                                            upperbound=self.maxContourArea)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        for contour in contours:
            # Find the center of each contour
            rect = cv2.minAreaRect(contour)
            centroid = rect[0]

            # Find the orientation of each contour
            points = np.int32(cv2.cv.BoxPoints(rect))
            edge1 = points[1] - points[0]
            edge2 = points[2] - points[1]

            if cv2.norm(edge1) > cv2.norm(edge2):
                angle = math.degrees(math.atan2(edge1[1], edge1[0]))
            else:
                angle = math.degrees(math.atan2(edge2[1], edge2[0]))

            if 90 < abs(Utils.normAngle(self.comms.curHeading) -
                        Utils.normAngle(angle)) < 270:
                angle = Utils.invertAngle(angle)

            samples.append({'centroid': centroid, 'angle': angle,
                            'area': cv2.contourArea(contour),
                            'category': category})

            if self.debugMode:
                Vision.drawRect(outImg, points)

    # Main processing function, should return (retData, outputImg)
    def gotFrame(self, img):
        outImg = None
        samples = list()
        site = dict()
        box = dict()
        rval = {'samples': samples, 'site': site, 'box': box}

        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        img = Vision.enhance(img)
        img = cv2.GaussianBlur(img, (5, 5), 0)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        if self.comms.visionMode == PickupVision.SAMPLES:
            cheeseImg = cv2.inRange(hsvImg, self.greenLoThresh, self.greenHiThresh)
            rockImg = cv2.inRange(hsvImg, self.redLoThresh1, self.redHiThresh1)
            rockImg |= cv2.inRange(hsvImg, self.redLoThresh2, self.redHiThresh2)

            cheeseImg = self.morphologyCheese(cheeseImg)
            rockImg = self.morphologyRock(rockImg)
            binImg = cheeseImg | rockImg

            if self.debugMode:
                outImg = cv2.cvtColor(binImg.copy(), cv2.COLOR_GRAY2BGR)

            self.findSamples('cheese', cheeseImg, samples, outImg)
            self.findSamples('rock', rockImg, samples, outImg)

            if self.debugMode:
                # Draw the centroid and orientation of each contour
                for sample in samples:
                    centroid = sample['centroid']
                    angle = sample['angle']
                    cv2.circle(outImg, (int(centroid[0]), int(centroid[1])),
                               5, (0, 0, 255))
                    startpt = centroid
                    gradient = np.deg2rad(angle)
                    endpt = (int(startpt[0] + 100 * math.cos(gradient)),
                             int(startpt[1] + 100 * math.sin(gradient)))
                    startpt = (int(startpt[0]), int(startpt[1]))
                    cv2.line(outImg, startpt, endpt, (255, 0, 0), 2)
        else:
            if self.comms.visionMode == self.SITE:
                binImg = cv2.inRange(hsvImg,
                                     self.yellowLoThresh2, self.yellowHiThresh2)
                binImg = binImg | cv2.inRange(hsvImg,
                                              self.yellowLoThresh3,
                                              self.yellowHiThresh3)
                binImg = self.morphology(binImg)
            elif self.comms.visionMode == self.BOX:
                binImg = cv2.inRange(hsvImg,
                                     self.yellowLoThresh, self.yellowHiThresh)
                binImg = self.morphologyBox(binImg)

            if self.debugMode:
                outImg = cv2.cvtColor(binImg.copy(), cv2.COLOR_GRAY2BGR)

            if self.comms.visionMode == self.SITE:
                contours = self.findContourAndBound(binImg.copy(), bounded=True,
                                                    bound=self.minSiteArea)
            elif self.comms.visionMode == self.BOX:
                contours = self.findContourAndBound(binImg.copy(), bounded=True,
                                                    bound=self.minBoxArea)
            if len(contours) > 0:
                if self.comms.visionMode == PickupVision.BOX:
                    centroids = map(lambda c: self.findCenter(c), contours)
                    meanX = np.mean(map(lambda c: c[0], centroids))
                    meanY = np.mean(map(lambda c: c[1], centroids))
                    centroid = (meanX, meanY)
                    box['centroid'] = centroid
                else:
                    largestContour = max(contours, key=cv2.contourArea)
                    rect = cv2.minAreaRect(largestContour)
                    centroid = rect[0]
                    site['centroid'] = centroid
                    # Find the orientation of each contour
                    #points = np.int32(cv2.cv.BoxPoints(cv2.minAreaRect(largestContour)))
                    #edge1 = points[1] - points[0]
                    #edge2 = points[2] - points[1]

                    #if cv2.norm(edge1) > cv2.norm(edge2):
                    #    angle = math.degrees(math.atan2(edge1[1], edge1[0]))
                    #else:
                    #    angle = math.degrees(math.atan2(edge2[1], edge2[0]))

                    #if 90 <  Utils.normAngle(angle) < 270:
                    #    angle = Utils.invertAngle(angle)
                    #site['angle'] = angle

                    if self.debugMode:
                        points = cv2.cv.BoxPoints(cv2.minAreaRect(largestContour))
                        Vision.drawRect(outImg, points)
                cv2.circle(outImg, (int(centroid[0]), int(centroid[1])),
                           7, (0, 0, 255), -1)

        if self.debugMode:
            # Draw the aiming rectangle
            midX = self.screen['width']/2.0
            midY = self.screen['height']/2.0
            maxDeltaX = self.screen['width']*0.03
            maxDeltaY = self.screen['height']*0.03
            cv2.rectangle(outImg,
                          (int(midX-maxDeltaX), int(midY-maxDeltaY)),
                          (int(midX+maxDeltaX), int(midY+maxDeltaY)),
                          (0, 255, 0), 1)

        return rval, outImg


def main():
    import rospy
    rospy.init_node("pickup_vision")
    cv2.namedWindow("output")
    image = cv2.imread("green_cheese.png")
    from comms import Comms
    visionFilter = PickupVision(Comms())
    _, outImg = visionFilter.gotFrame(image)
    cv2.imshow("output", outImg)
    cv2.waitKey()
