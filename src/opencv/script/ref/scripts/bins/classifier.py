#!/usr/bin/env python

import cv2
import random
import numpy as np

class Generator():
    loHSVThresh1 = (0, 0, 0)
    hiHSVThresh1 = (30, 255, 255)
    loHSVThresh2 = (165, 0, 0)
    hiHSVThresh2 = (180, 255, 255)

    def __int__(self):
        pass

    def generateClass(self, img, outfile):
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        binImg = cv2.inRange(hsvImg, self.loHSVThresh1, self.hiHSVThresh1)
        binImg |= cv2.inRange(hsvImg, self.loHSVThresh2, self.hiHSVThresh2)

        contours,_ = cv2.findContours(binImg.copy(),
                                    cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_NONE)
        assert(len(contours) == 1)

        np.save(outfile, contours[0])
        return cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)

def main():
    cv2.namedWindow("display")
    generator = Generator()
    img = cv2.imread("res/4.png")
    cv2.imshow("display", generator.generateClass(img, "res/4"))
    cv2.waitKey()

if __name__ == "__main__":
    main()
