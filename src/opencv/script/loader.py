#!/usr/bin/env python

from matplotlib import pyplot as plt
import numpy as np 
import cv2
import os
import sys
import rospy
from utils.utils import Utils
from sensor_msgs.msg import Image

IMG_PATH = 'img/pole/pole47.jpg'
#front_cam_topic = '/bottomcam/camera/image_raw_jin'
front_cam_topic = '/front_camera/camera/image_raw_jin'
bot_cam_topic = '/bottom_camera/camera/image_raw_jin'
front_out = '/vision_filter/front'
bottom_out = '/vision_filter/bottom'

class Vid:

    def __init__(self):
        rospy.init_node('ahmumu_vid') 
        rospy.loginfo('Node init')
        self.cam_sub = rospy.Subscriber(front_cam_topic, Image, self.cam_cb)
        self.vision_filter_pub = rospy.Publisher(front_out, Image, queue_size=10)

    def cam_cb(self, rosimg):
        cvimg = Utils.rosimg2cv(rosimg)
        out = Img(cvimg).output
        self.vision_filter_pub.publish(Utils.cv2rosimg(out))

class Img:

    def __init__(self, src):
        #Metadata
        self.src = src    
        self.height = self.src.shape[0]
        self.width = self.src.shape[1]
        self.size = self.src.size
        #Typical color models used
        self.hsv_img = cv2.cvtColor(self.src, cv2.COLOR_BGR2HSV)
        self.lab_img = cv2.cvtColor(self.src, cv2.COLOR_BGR2LAB)
        self.gray_img = cv2.cvtColor(self.src, cv2.COLOR_BGR2GRAY)
        #Histograms
        self.gray_hist = cv2.calcHist([self.gray_img], [0], None, [256], [0, 256])#None can be replaced by mask
        #Color-constancy output 
        self.usm = Utils.usm(self.src)
        self.iace = Utils.iace(self.src)
        self.hybrid_clahe = Utils.hybrid_clahe(self.src)
        layer1 = np.hstack((self.src, self.iace))
        layer2 = np.hstack((self.usm, self.hybrid_clahe))
        self.output = np.vstack((layer1, layer2))
         

    def displayImg(self, img):
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.imshow('image', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def getWeightMaps(img):
    white_balanced = Utils.iace(img.src)
    blur = Utils.median_blur(white_balanced,7)
    contr = Utils.clahe(blur)
    luminance = Utils.luminance_map(img.src)
    chrom = Utils.chromatic_color_map(img.src)
    saliency = Utils.saliency_color_map(contr)
    exposedness = Utils.exposedness_color_map(img.src)
    local_contrast = Utils.local_color_contrast(img.src)
    img.displayImg(np.hstack((luminance[0],chrom[0],saliency[0],exposedness[0],local_contrast[0])))
    blended = Utils.blending(contr, local_contrast)
    return blended

def batch_write(path, output_path):
    files = filter(lambda f:os.path.isfile(os.path.join(path, f)), os.listdir(path))
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    for i in files:
        full_path = path + "/" +  i
        print(full_path)
        img = Img(full_path)
        fr = Utils.hybrid_clahe(img.src)
        tmp = np.hstack((img.src, fr))
        cv2.imwrite(output_path+'/'+i, tmp)
    
    

if __name__ == '__main__':
    #vid = Vid()
    img = Img(cv2.imread(IMG_PATH))
    tmp = Utils.hybrid_clahe(img.src)
    img.displayImg(np.hstack((img.src,tmp)))
    #rospy.spin()
