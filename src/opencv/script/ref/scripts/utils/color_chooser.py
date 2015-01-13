#!/usr/bin/env python

import cv2
import numpy as np

canvas = np.zeros((10, 181, 3), dtype=np.uint8)

def redraw(canvas, loBound, upBound):
    for i in range(181):
        if i == loBound or i == upBound:
            canvas[:, i, :] = 0
        else:
            canvas[:, i, 0] = i
            canvas[:, i, 1] = 220
            canvas[:, i, 2] = 220

def display():
    global canvas
    canvas = cv2.cvtColor(canvas, cv2.COLOR_HSV2BGR)
    img = cv2.resize(canvas, (181*4, 50))

    cv2.imshow("color_chooser", img)

def onChange(data):
    global canvas
    redraw(canvas,
           cv2.getTrackbarPos("loH", "color_chooser"),
           cv2.getTrackbarPos("hiH", "color_chooser"))
    display()


if __name__ == "__main__":
    cv2.namedWindow("color_chooser")
    cv2.createTrackbar("loH", "color_chooser", 0, 180, onChange)
    cv2.createTrackbar("hiH", "color_chooser", 0, 180, onChange)

    redraw(canvas, 0, 0)
    display()
    # Wait for esc key
    while cv2.waitKey(0) != 27:
        pass
