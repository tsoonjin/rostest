#!/usr/bin/env python
import numpy as np
import os

class Config():
    def __init__(self):
        self.screen_w = 640
        self.screen_h = 480
        self.center_w = self.screen_w/2
        self.center_h = self.screen_h/2
        self.compassTopic = "/euler"
        self.camTopic = "/front_camera/camera/image_raw"
        self.depthTopic = "/depth"
        self.pubTopic = "/Vision/image_filter_red"
        self.pubTopic2 = "/Vision/image_filter_green"
        self.enhanceTopic = "/Vision/image_filter_enhance"
        self.pubTopic3 = "/Vision/image_filter"
        self.red_min = np.array([0,200,0], dtype='uint8')
        self.red_max = np.array([8,255,255], dtype='uint8')
        self.green_min = np.array([50,200,0], dtype='uint8')
        self.green_max = np.array([70,255,255], dtype='uint8')
        self.forward_step = 1.0
        self.sm_y = -0.5
        self.sm_x = 0.9
        self.passLimit = 0.3
        dir = os.path.dirname(os.path.abspath(__file__))
        self.path = dir + "/bag/frame1111.jpg"
        self.defaultDepth = 2.0
        self.mult_x = 5.0
        self.mult_y = 2.5
        
        

if __name__ == "__main__":
    pass
