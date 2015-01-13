#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

import math

class Utils():
    bridge = CvBridge()

    @staticmethod
    def rosimg2cv(ros_img):
        try:
            frame = Utils.bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        return frame

    @staticmethod
    def cv2rosimg(cv_img):
        try:
            return Utils.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)


    @staticmethod 
    def toBGR(img, flag):
        if flag is 'gray':
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if flag is 'hsv':
            return cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        if flag is 'lab':
            return cv2.cvtColor(img, cv2.COLOR_LAB2BGR)
         
    """ Gaussian Blur 
    @param: src ksize sigmaX sigmaY 
    ksize -- (x, y) must be odd
    sigma -- if 0 calculated from kernel size. determines width of the gaussian function 
          -- if only x pecified, y will take the same value

    """
    @staticmethod
    def gaussian_blur(img, ksize, sigma=0):
        return cv2.GaussianBlur(img, ksize, sigma)
        
    """ Median Blur 
    Replaces with median value of the area. Good for salt & pepper noise
    @param: src ksize

    """
    @staticmethod
    def median_blur(img, ksize):
        return cv2.medianBlur(img, ksize)

    """ Bilateral Filter
    Remove noise while preserving edge. CAUTION: slow
    @param: src d sigmaColor sigmaSpace
    d -- diamter of pixel neighborhood
    sigmaColor -- larger val -> farther color will be mixed-in
    sigmaSpace -- largeer val -> farther pixel influence other

    """
    @staticmethod
    def bilateral_filter(img, d, sigmaColor, sigmaSpace):
        return cv2.bilateralFilter(img, d, sigmaColor, sigmaSpace)

    """ Calc Histogram 
    @param: src  channels mask histSize ranges
    src -- wrapped in list. e.g [img]
    color -- ('k') for black color line 

    """

    @staticmethod
    def calc_hist(src):
        h = np.zeros(src.shape)
        bins = np.arange(256).reshape(256,1)
        color = [ (255,0,0),(0,255,0),(0,0,255) ]

        for ch, col in enumerate(color):
            hist_item = cv2.calcHist([src],[ch],None,[256],[0,255])
            cv2.normalize(hist_item,hist_item,0,255,cv2.NORM_MINMAX)
            hist=np.int32(np.around(hist_item))
            pts = np.column_stack((bins,hist))
            cv2.polylines(h,[pts],False,col)
        h=np.flipud(h)
        return h

    """ Get ROI
    format -- [row, column] 
    """
    @staticmethod 
    def getROI(img, x_begin, x_end, y_begin, y_end):
        return img[y_begin:y_end, x_begin:x_end]

    """ Split into channels
    Very costly operation use with care

    """
    @staticmethod
    def split(img):
        return cv2.split(img)

    """Create Image
    @param: height width channels
    black image
    """
    @staticmethod
    def create_img(height, width, channel):
        return np.zeros((height, width, channel), np.uint8)

    """Create Kernel 
    @param: x y shape
    shape -- rect, ellipse, cross, ...

    """
    @staticmethod
    def createKernel(x, y, shape):
        if shape is 'rect':
            return cv2.getStructuringElement(cv2.MORPH_RECT, (x, y))
        if shape is 'ellipse':
            return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (x,y))

    """Morphology 
    @param: src kernel iterations type
    style -- dilate, erode, open, close
    dilate -- expand foreground obj. pixel is 1 if at least one pixel in kernel is 1
    open -- erode then dilate. removal of noise
    close -- dilate then erode. close of small holes inside
    can also use cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    
    """
    @staticmethod
    def morphology(img, kernel, itr, style):
        if style is "dilate":
            return cv2.dilate(img, kernel, itr)
        if style is "erode":
            return cv2.erode(img, kernel, itr)

    """Color Thresholding 
    @param: src min max 
    min, max -- take in np.array([minR,minG,minB], dtype='uint8')
    return grayscale image
    
    """
    @staticmethod
    def color_threshold(img, min_thresh, max_thresh):
        return cv2.inRange(img, np.array(min_thresh, dtype='uint8'), np.array(max_thresh, dtype='uint8'))

    """Black&White Thresholding 
    @param: src thresh maxVal flag 
    maxVal -- used when pixel exceeded thresh
    cv2.THRESH_TRUNCATE -- uses thresh as val instead of maxVal when exceeded 
    cv2.THRESH_TOZERO -- when exceeded thresh, 0
    return retval(used for Otsu binarization), thresholded img
     
    """
    @staticmethod
    def threshold(img, thresh, maxVal, flag):
        return cv2.threshold(img, thresh, maxVal, flag)

    """Adaptive Thresholding 
    @param: src maxVal adaptiveMethod type blocksize c
    returns only thresholded img
    adaptiveMethod -- cv2.ADAPTIVE_THRESH_MEAN_C/GAUSSIAN_C 
    blocksize -- determines size of local thresholding 
    c -- a constant used to minus of mean
    """
    @staticmethod
    def adaptive_threshold(img, maxVal, flag, block, c):
        return cv2.adaptiveThreshold(img, maxVal, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, flag, block, c)

    """Otsu's Binarization 
    Identifies a thresholded val between two peaks in histogram
    @param: img 0 255 type+cv2.THRESH_OTSU

    """
    @staticmethod
    def otsu_threshold(img, flag):
        return cv2.threshold(img, 0, 255, flag+cv2.THRESH_OTSU)

    """Resize
    @param: src (height, width)

    """
    @staticmethod 
    def resize(src, size):
        return cv2.resize(src, size)


    """Image Pyramid 
    @param: img 
    cv2.pyrDown -- lower res 
    cv2.pyrUp -- higher res

    """
    @staticmethod 
    def pyramid(src, type):
        if type is "up":
            return cv2.pyrUp(src)
        elif type is "down":
            return cv2.pyrDown(src)

    """Drawing
    @param: canvas coordinates color 
    coordinates -- (x,y) 
    color -- (r,g,b)
    thickness -- if negative filled poligon will be drawn

    """

    @staticmethod
    def draw_rect(canvas, top_left, bot_right, color, thickness):
        cv2.rectangle(canvas, top_left, bot_right, color, thickness)

    @staticmethod 
    def draw_circle(src, center, rad, color, thickness):
        cv2.circle(src, center, rad, color, thickness)

    @staticmethod 
    def text(canvas, text, org, fontScale, color):
        cv2.putText(canvas, text, org, cv2.FONT_HERSHEY_SIMPLEX, fontScale, color, cv2.CV_AA)

    """Find&Draw Contour 
    @param: thresh hierr approx
    hierr -- ways to store contours. cv2.RETR_EXTERNAL take the external of each hierr only 
    approx -- cv2.CHAIN_APPROX_SIMPLE(store minimal pts needed)
    returns contours, hierr
    contourIdx -- draws the kth contour
    contours must be wrapped as list even though only one 

    """
    @staticmethod 
    def find_contour(img, hierr, approx):
        return cv2.findContours(img, hierr, approx)

    @staticmethod 
    def draw_contour(canvas, contours, contourIdx, color, thickness):
        cv2.drawContours(canvas, contours, contourIdx, color, thickness)

    @staticmethod
    def moments(cnt):
        return cv2.moments(cnt)

    @staticmethod 
    def centroid(mom):
        x = int((mom['m10']+0.0001)/(mom['m00']+0.0001))
        y = int((mom['m01']+0.0001)/(mom['m00']+0.0001))
        return (x, y)

    @staticmethod 
    def area(cnt):
        return cv2.contourArea(cnt)

    @staticmethod
    def perimeter(cnt):
        return cv2.arcLength(cnt, True)#second param indicates whether contour is closed or not

    @staticmethod 
    def min_rect(cnt):
        rect = cv2.minAreaRect(cnt) #rect is in the form center(x,y), (width, height), angle of rotation
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        return box

    @staticmethod 
    def min_circle(cnt):
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        center = (int(x),int(y))
        radius = int(radius)
        return (center, radius)

    """Adding images
    @param: img1 alpha img2 beta gamma 
    alpha, beta -- weigth in final image
    gamma -- just a constant to be added 
    must be same depth i.e both 3 channels 

    """
    @staticmethod
    def blend(img1, a, img2, b, g):
        return cv2.addWeighted(img1, a, img2, b, g)

    """IACE 
    image adaptive contrast enhancement 

    """
    @staticmethod 
    def clahe(img):
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        b,g,r = cv2.split(img)
        b = clahe.apply(b)
        g = clahe.apply(g)
        r = clahe.apply(r)
        final = cv2.merge((b,g,r))
        hsv = cv2.cvtColor(final, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(hsv)
        v = clahe.apply(v)
        final2 = cv2.cvtColor(cv2.merge((h,s,v)), cv2.COLOR_HSV2BGR)
        return final2

    @staticmethod
    def util_iace(channel):
       min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(channel)
       channel_ = (channel - min_val)/(max_val-min_val)*255.0  
       return channel_
       #return cv2.normalize(channel_, 0, 255, cv2.NORM_MINMAX)

    """Image Adaptive Contrast Enhancement
    P_out = (P_in - c)/(d-c)*255
    d = max_val
    c = min_val

    """
    @staticmethod
    def iace(img):
       b,g,r = cv2.split(img) 
       b_ = Utils.util_iace(b) 
       g_ = Utils.util_iace(g)
       r_ = Utils.util_iace(r) 
       out = cv2.merge((np.uint8(b_),np.uint8(g_),np.uint8(r_))) #scale up to 255 range
       Utils.text(out, 'IACE', (300,300), 3, (0,0,255))
       return out

    @staticmethod
    def denoise(img): #grayscale only
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        return cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)

    @staticmethod
    def illum_norm(y):
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        return clahe.apply(y)

    @staticmethod
    def french_preprocess(img):
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        y, cr, cb = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB))
        homo = Utils.illum_norm(y)
        denoised = Utils.denoise(homo)
        ansio = cv2.GaussianBlur(denoised, (3,3), 1)
        bgr = cv2.cvtColor(cv2.merge((ansio,cr,cb)), cv2.COLOR_YCR_CB2BGR)
        b,g,r = cv2.split(bgr)
        b = clahe.apply(b)
        g = clahe.apply(g)
        r = clahe.apply(r)
        return cv2.merge((b,g,r))
        

    @staticmethod
    def usm(img):
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        b,g,r = cv2.split(img)
        min_b, max_b, _, _ = cv2.minMaxLoc(b)
        min_g, max_g, _, _ = cv2.minMaxLoc(g)
        min_r, max_r, _, _ = cv2.minMaxLoc(r)
        mean_b = cv2.mean(b)[0]
        mean_g = cv2.mean(g)[0]
        mean_r = cv2.mean(r)[0]
        A = mean_b/mean_r
        B = mean_b/mean_g
        r = A*r
        #r = cv2.normalize(r, 0, 255, cv2.NORM_MINMAX)*255
        g = B*g
        g = cv2.normalize(g, 0, 255, cv2.NORM_MINMAX)*255
        #red contrast stretch to max 
        r = ((r - min_r)*(255 - min_r)/(max_r - min_r))+min_r
        r_ = cv2.normalize(r, 0, 255, cv2.NORM_MINMAX)*255
        #blue contrast stretch to min
        b = ((b - 0)*(max_b - 0)/(max_b - min_r))
        b_ = cv2.normalize(b, 0, 255, cv2.NORM_MINMAX)*255
        #green contrast stretch to boths sides
        g = ((g - 0)*(255)/(max_g - min_g))
        g_ = cv2.normalize(g, 0, 255, cv2.NORM_MINMAX)*255
        tmp1 = cv2.merge((np.uint8(b_),np.uint8(g_),np.uint8(r_)))
        hsv = cv2.cvtColor(tmp1, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(hsv)
        s = clahe.apply(s)
        v = clahe.apply(v)
        out =  cv2.cvtColor(cv2.merge((h,s,v)), cv2.COLOR_HSV2BGR)
        Utils.text(out, 'USM', (300,300), 3, (0,0,255))
        return out
        
    @staticmethod
    def hybrid_clahe(img):
        img = cv2.medianBlur(img, 9)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(3,3))
        h,l,s = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS))
        s = clahe.apply(s)
        l = clahe.apply(l)
        hls2bgr = cv2.cvtColor(cv2.merge((h,l,s)), cv2.COLOR_HLS2BGR)
        b_,g_,r_ = cv2.split(hls2bgr)
        b,g,r = cv2.split(img)
        r = clahe.apply(r)
        g = clahe.apply(g)
        b = clahe.apply(b)
        """
        map(lambda x:cv2.pow(x,2), [b,g,r,b_,g_,r_])
        b = cv2.pow(np.float32(b+b_),0.5)
        g = cv2.pow(np.float32(g+g_),0.5)
        r = cv2.pow(np.float32(r+r_),0.5)
        """
        rgb = cv2.merge((b,g,r))
        out = cv2.addWeighted(hls2bgr, 0.4, rgb, 0.4,0)
        Utils.text(out, 'HYBRID_CLAHE', (300,300), 3, (0,0,255))
        return out

    @staticmethod 
    def luminance_map(img):
        emptyMap = np.zeros(img.shape[:2], dtype=np.uint8)
        h,l,s = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS))
        b,g,r = cv2.split(img)
        y, x = img.shape[:2]
        for i in xrange(y):
            for j in xrange(x):
                b_item = b.item((i,j))
                g_item = g.item((i,j))
                r_item = r.item((i,j))
                l_item = l.item((i,j))
                std = np.std([b_item, g_item, r_item, l_item])
                emptyMap[i][j] = std
        print "Luminance Map generated"
        return [emptyMap, emptyMap.astype(np.float32, copy=False)/255.0]

    @staticmethod
    def chromatic_color_map(img):
        emptyMap = np.zeros(img.shape[:2], dtype=np.uint8)
        h,s,v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
        b,g,r = cv2.split(img)
        y, x = img.shape[:2]
        img.itemset((0,0,2), 1)
        for i in xrange(y):
            for j in xrange(x):
                b_item = b.item((i,j))
                g_item = g.item((i,j))
                r_item = r.item((i,j))
                s_item = s.item((i,j))
                std = np.std([b_item, g_item, r_item, s_item])
                emptyMap.itemset((i,j), std)
        print "Chromatic Map generated"
        return [emptyMap, emptyMap.astype(np.float32, copy=False)/255.0]

    @staticmethod
    def chromatic_single_map(img,channel):
        emptyMap = np.zeros(img.shape[:2], dtype=np.uint8)
        h,s,v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
        y, x = img.shape[:2]
        for i in xrange(y):
            for j in xrange(x):
                chan_item = channel.item((i,j))
                s_item = s.item((i,j))
                std = np.std([chan_item, s_item])
                emptyMap[i][j] = std
        return emptyMap

    @staticmethod
    def saliency_color_map(img):
        b,g,r = cv2.split(img)
        b = Utils.saliency_map(b)[1]
        g = Utils.saliency_map(g)[1]
        r = Utils.saliency_map(r)[1]
        return (b+g+r)/3.0

    @staticmethod 
    def saliency_map(chan):#applicable to different channels
        emptyMap = np.zeros(chan.shape[:2], dtype=np.uint8)
        y,x = chan.shape[:2]
        chan_mean = np.mean(chan)
        blur = cv2.GaussianBlur(chan, (5,5), 0)
        for i in xrange(y):
            for j in xrange(x):
                final = abs(chan_mean - blur[i][j])   
                emptyMap[i][j] = final 
        print "Saliency Map generated"
        return [emptyMap, emptyMap.astype(np.float32, copy=False)/255.0]

    @staticmethod 
    def exposedness_color_map(img):
        b,g,r = cv2.split(img)
        b = Utils.exposedness_map(b)[1]
        g = Utils.exposedness_map(g)[1]
        r = Utils.exposedness_map(r)[1]
        fi = (b+g+r)/3.0
        return fi
    
    @staticmethod
    def exposedness_map(chan):
        e = 2.718
        emptyMap = np.zeros(chan.shape[:2], dtype=np.uint8)
        y, x = chan.shape[:2]
        for i in xrange(y):
            for j in xrange(x):
                std = 0.25
                gauss = -(float(chan[i][j]/255.0) - 0.5)**2/(2*(std**2))
                sol = e**gauss
                emptyMap[i][j] = sol*255
        print "Exposedness Map generated"
        return [emptyMap, emptyMap.astype(np.float32, copy=False)/255.0]

    @staticmethod
    def local_color_contrast(img):
        b,g,r = cv2.split(img)
        b = Utils.local_contrast(b)[1]
        g = Utils.local_contrast(g)[1]
        r = Utils.local_contrast(r)[1]
        return (b+g+r)/3.0

    @staticmethod 
    def local_contrast(img):
        #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(img,  (5,5), 0)
        final = img - blur
        print "Local Contrast Map generated"
        return [final, final.astype(np.float32, copy=False)/255.0]

    @staticmethod
    def norm_weight(y, x, weight_maps):
        emptyMap = np.zeros(weight_maps[0].shape[:2], dtype=np.float32)
        for i in xrange(y):
            for j in xrange(x):
                ls = [a[i][j] for a in weight_maps]
                emptyMap[i][j] = np.mean(ls)
        return [np.uint8(emptyMap)*255, emptyMap]

    @staticmethod 
    def blending(img, weight):
        b,g,r = cv2.split(img)
        for i in xrange(img.shape[0]):
            for j in xrange(img.shape[1]):
                b[i][j] = b[i][j] * weight[i][j]
                g[i][j] = g[i][j] * weight[i][j]
                r[i][j] = r[i][j] * weight[i][j]
        final = cv2.merge((np.uint8(b), np.uint8(g), np.uint8(r)))
        return final

    @staticmethod 
    def Laplacian(img):
        return cv2.Laplacian(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.CV_64F)





