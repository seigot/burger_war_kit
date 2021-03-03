#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import time
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage
import cv2

class LookatEnemy:
    width_img = 640
    height_img = 480
    ch_img = 3
    shape_img = height_img, width_img,ch_img
    h_thrshld_uppr = 90 #120
    h_thrshld_lwr  = 60 #42
    s_thrshld_lwr  = 100
    hight_bbox_thrshld_lwr = height_img * 0.25 #0.15 #0.2


    notfound = 404
    notfound_flg = False
    cam_img = np.zeros(shape_img)
    green_img = np.zeros(shape_img)
    rect_img = np.zeros(shape_img)

    def __init__(self):
        self.camcmpress_sub = rospy.Subscriber('image_raw/compressed', CompressedImage, self.GetCamCmprssImgCallback, queue_size=1)
        self.conv_start_time = time.time()
        self.conv_now_time = self.conv_start_time
        self.conv_elapsed_time = self.conv_now_time - self.conv_now_time
        self.lookatEnemy_pub = rospy.Publisher('lookatEnemy', Int32, queue_size=1)

    def GetCamCmprssImgCallback(self, data):
        self.conv_now_time = time.time()
        self.conv_elapsed_time = self.conv_now_time - self.conv_start_time
        if self.conv_elapsed_time > 0.033:
            self.conv_start_time = self.conv_now_time
            np_arr = np.fromstring(data.data, np.uint8)
            self.cam_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            g_diff_px_x = self.CheckandLookatGreenCircle(self.cam_img)
            self.lookatEnemy_pub.publish(g_diff_px_x)

    def CheckandLookatGreenCircle(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        mask = np.zeros(h.shape, dtype=np.uint8)
        mask[(self.h_thrshld_lwr < h) & (h < self.h_thrshld_uppr) & (s > self.s_thrshld_lwr)] = 255
        self.green_img, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        circles = []
        if len(contours) > 0:
            maxCont=contours[0]
            for c in contours:
                if len(maxCont)<len(c):
                    maxCont = c
            x_bbox, y_bbox, w_bbox, h_bbox = cv2.boundingRect(maxCont)

            if h_bbox > self.hight_bbox_thrshld_lwr:
                self.notfound_flg = False
                self.rect_img = cv2.rectangle(image,(x_bbox, y_bbox),(x_bbox+w_bbox,y_bbox+h_bbox),(0,255,0),2)
                return (x_bbox + w_bbox / 2.0) - self.width_img / 2.0
            else:
                if not self.notfound_flg:
                    self.notfound_flg = True
                    self.rect_img = np.zeros(self.shape_img)
                return self.notfound
        else:
            if not self.notfound_flg:
                self.notfound_flg = True
                self.rect_img = np.zeros(self.shape_img)
            return self.notfound

if __name__ == '__main__':
    rospy.init_node('lookatEnemy', anonymous=True)
    lookatEnemy = LookatEnemy()
    #cv2.namedWindow("Camera Image")

    r = rospy.Rate(15)
    while not rospy.is_shutdown():
        #cv2.imshow("Camera Image", lookatEnemy.rect_img)
        cv2.waitKey(1)
        r.sleep()
