import cv2
import numpy as np
import math


class Vision():
    SHAPE_TRI = 0
    SHAPE_RECT = 1
    SHAPE_SQU = 2
    SHAPE_PENTA = 3
    SHAPE_HEXA = 4
    SHAPE_CONCAVE = 5
    SHAPE_CIR = 6

    _angleBounds = {'rect': (88, 92), 'penta': (-0.34, -0.27),
                  'hexa': (-0.55, -0.45)}

    @staticmethod
    def drawRect(img, pts, color=(0, 0, 255)):
        points = np.int32(pts)
        for i in range(4):
            pt1 = (points[i][0], points[i][1])
            pt2 = (points[(i+1) % 4][0], points[(i+1) % 4][1])
            cv2.line(img, pt1, pt2, color, 2)

    @staticmethod
    def drawPoly(img, pts, color=(0, 0, 255)):
        points = np.int32(map(lambda p: p[0], pts))
        numPoints = len(pts)
        for i in range(numPoints):
            pt1 = (points[i][0], points[i][1])
            pt2 = (points[(i+1) % numPoints][0], points[(i+1) % numPoints][1])
            cv2.line(img, pt1, pt2, color, 2)

    @staticmethod
    def shapeEnum2Str(shapeEnum):
        if shapeEnum is None:
            return 'UNKNOWN'

        if shapeEnum == Vision.SHAPE_TRI:
            return 'TRI'
        if shapeEnum == Vision.SHAPE_RECT:
            return 'RECT'
        if shapeEnum == Vision.SHAPE_SQU:
            return 'SQU'
        if shapeEnum == Vision.SHAPE_PENTA:
            return 'PENTA'
        if shapeEnum == Vision.SHAPE_HEXA:
            return 'HEXA'
        if shapeEnum == Vision.SHAPE_CIR:
            return 'CIR'
        if shapeEnum == Vision.SHAPE_CONCAVE:
            return 'CONCAVE'

        return 'UNKNOWN'

    @staticmethod
    def angle(pt1, pt2, pt0):
        """ Find angle around p0 made by p1 and p2
            p1, p2, p0 are 2D points: (x-coord, y-coord) """
        dx1 = pt1[0] - pt0[0]
        dy1 = pt1[1] - pt0[1]
        dx2 = pt2[0] - pt0[0]
        dy2 = pt2[1] - pt0[1]
        cosine = float(dx1*dx2 + dy1*dy2)/(math.sqrt(dx1*dx1 + dy1*dy1) *
                                           math.sqrt(dx2*dx2 + dy2*dy2) + 1e-4)
        return math.degrees(math.acos(abs(cosine)))

    @staticmethod
    def isRectangle(contour, epsilon=15):
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) != 4:
            return False, approx

        vtc = len(approx)
        # Find the cosine for angles of the polygon
        angles = list()
        for i in range(2, vtc+1):
            angles.append(Vision.angle(approx[i % vtc][0],
                                       approx[i-2][0],
                                       approx[i-1][0]))
        # Sort the angles cosine from smallest to largest
        sorted(angles)

        # Get the smallest and largest angle cosine
        minAngle = angles[0]
        maxAngle = angles[-1]

        if minAngle >= Vision._angleBounds['rect'][0] and \
           maxAngle <= Vision._angleBounds['rect'][1]:
            return True, approx
        return False, approx

    @staticmethod
    def shadesOfGray(img):
        """ Implementation of the Shades of Gray algorithm, which is a
        combination of Gray World algorithm and MaxRGB algorithm, which are
        used to normalize color images to achieve color constancy """

        inB, inG, inR = cv2.split(img)
        avgR = np.mean(inR)
        avgG = np.mean(inG)
        avgB = np.mean(inB)
        avgGray = np.mean((avgB, avgG, avgR))

        if avgB == 0:
            outB = inB
        else:
            outB = (avgGray/avgB)*inB

        if avgG == 0:
            outG = inG
        else:
            outG = (avgGray/avgG)*inG

        if avgR == 0:
            outR = inR
        else:
            outR = (avgGray/avgR)*inR

        maxRGB = (np.max(outR), np.max(outG), np.max(outB))
        factor = np.max(maxRGB)
        if factor > 1:
            outR = 255*outR/factor
            outG = 255*outG/factor
            outB = 255*outB/factor

        outImg = cv2.merge((np.uint8(outB), np.uint8(outG), np.uint8(outR)))
        return outImg

    @staticmethod
    def illuminanceMask(img, threshVal):
        grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grayImg = cv2.equalizeHist(grayImg)
        return cv2.threshold(grayImg, threshVal, 255, cv2.THRESH_BINARY)[1]

    @staticmethod
    def enhance(img):
        blurImg = cv2.GaussianBlur(img, ksize=(0, 0), sigmaX=10)
        enhancedImg = cv2.addWeighted(img, 2.5, blurImg, -1.5, 0)
        return enhancedImg


def main():
    img = cv2.imread("bot_common/shapes.png")
    grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edgeImg = cv2.Canny(grayImg, 0, 50, 5)
    contours, _ = cv2.findContours(edgeImg, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_NONE)
    for contour in contours:
        shape = Vision.detectShape(contour)
        pos = (cv2.minAreaRect(contour)[0])
        pos = (int(pos[0]), int(pos[1]))
        cv2.putText(img, Vision.shapeEnum2Str(shape), pos,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.imshow("display", img)
    cv2.waitKey()
