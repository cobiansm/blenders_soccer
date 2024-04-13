#!/usr/bin/env python2.7
#coding=utf-8

import rospy
import sensor_msgs
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import geometry_msgs
from geometry_msgs.msg import Point
import math
import std_msgs
from std_msgs.msg import Bool
import time


gradient = lambda x1,y1,x2,y2: (y2-y1)/(x2-x1)


class HeadControl:
    def __init__(self):
        rospy.init_node('PD_Head_Control')
        self.image_pub = rospy.Publisher('/Vision_node', sensor_msgs.msg.Image, queue_size=1)
        self.error_pub = rospy.Publisher('/error', geometry_msgs.msg.Point, queue_size=1)
        self.position_pub = rospy.Publisher('/position', geometry_msgs.msg.Point, queue_size=1)
        self.ball_pub = rospy.Publisher('/find_ball', std_msgs.msg.Bool, queue_size=1)
        self.turnNsearch_pub = rospy.Publisher('/turnNsearch', std_msgs.msg.Bool, queue_size=1)

        self.find_ball = Bool()
        self.turn_search = Bool()

        self.bridge = CvBridge()

        subimg = rospy.Subscriber("/usb_cam/image_raw", Image, self.center_callback)
        sub_center = rospy.Subscriber("/BallCenter", Point, self.callbackCenter)

        self.ux0 = 0
        self.ux0 = 0

        self.ux1 = 0
        self.uy1 = 0

        self.kpx = 0.5
        self.kdx = 0
        self.kix = 0

        self.kpy = 0.5
        self.kdy = 0
        self.kiy = 0

        self.errx0 = 0
        self.errx1 = 0
        self.errx2 = 0

        self.erry0 = 0
        self.erry1 = 0
        self.erry2 = 0

        self.center = Point()

        self.Tm = 0.1
        self.t = 0

        self.now = 0
        self.start_search = 0
        self.end_search = False

        self.noball_start_time = 0
        self.noball_now_time = 0
        self.noball_end_time = False

        self.kernel = np.ones((5,5),np.uint8)

    def center_callback(self, img_msg):

        rospy.loginfo(img_msg.header)

        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
            #cv_img = cv2.flip(cv_img,1)
        except CvBridgeError:
            rospy.logerr("CvBridge Error")

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)

        nR = np.matrix(blurred[:,:,0])
        nG = np.matrix(blurred[:,:,1])
        nB = np.matrix(blurred[:,:,2])

        color = cv2.absdiff(nG,nR)

        _, umbral = cv2.threshold(color,50,255,cv2.THRESH_BINARY)

        mask = cv2.erode(umbral,np.ones((35,35), np.uint8))
        mask = cv2.dilate(mask, None, iterations=2)
        
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        xb,yb,wb,hb = cv2.boundingRect(opening)
        self.xball = int(xb+(wb/2))
        self.yball = int(yb+(hb/2))

        cv2.rectangle(frame, (xb,yb), (xb+wb,yb+hb), (255,0,0), 8)
        cv2.circle(frame, (self.xball, self.yball), 5,  (255,255,255), 5)

        self.area = wb*hb
        print(self.area)

        final_img = self.bridge.cv2_to_imgmsg(frame, "rgb8")
        self.image_pub.publish(final_img)
    
    def callbackCenter(self, ball_center):

        self.center.x = ball_center.x
        self.center.y = ball_center.y

    def cal_err(self):
        x_center = 320
        y_center = 240

        #errorx = x_center - self.center.x
        #errory = y_center - self.center.y

        errorx = x_center - self.xball
        errory = y_center - self.yball

        errorx = errorx * 70/x_center
        errory = errory * 70/y_center

        point = Point()
        point.x = errorx
        point.y = errory

        error_point = Point()
        error_point.x = errorx
        error_point.y = errory

        self.error_pub.publish(error_point)

        return point

    def control(self):
        self.turn_search.data = False
        self.turnNsearch_pub.publish(self.turn_search)

        point = self.cal_err()

        self.errx0 = point.x
        self.erry0 = point.y

        if self.errx0 != 70 and self.erry0 != 70:
            
            self.noball_start_time = 0
            self.noball_now_time = 0
            self.noball_end_time = False

            self.start_search = 0

            ux = self.ux1 + (self.kpx + self.kdx/self.Tm)*self.errx0 + (-self.kpx + self.kix*self.Tm - 2*self.kdx/self.Tm)*self.errx1 + (self.kdx/self.Tm)*self.errx2
            #ux = ux1 + (Kpx + Kdx/Tm)*errx0 + (-2*Kdx/Tm)*errx1 + (-Kpx + Kdx/Tm)*errx2
            uy = self.uy1 + (self.kpy + self.kdy/self.Tm)*self.erry0 + (-self.kpy + self.kiy*self.Tm - 2*self.kdy/self.Tm)*self.erry1 + (self.kdy/self.Tm)*self.erry2
            #uy = uy1 + (Kpy + Kdy/Tm)*erry0 + (-2*Kdy/Tm)*erry1 + (-Kpy + Kdy/Tm)*erry2

            if ux >= 70: ux = 70
            elif ux <= -70: ux = 70

            if uy >= 0: uy = -5
            elif uy <= -70: uy = -70

            self.ux1 = ux
            self.errx1 = self.errx0
            self.errx2 = self.errx1

            self.uy1 = uy
            self.erry2 = self.erry1
            self.erry1 = self.erry0

            #if ux > 3 or ux < -3:

            if (self.errx0 < -5 or self.errx0 > 5) or (self.erry0 < -5 or self.erry0 > 5):

                ux = (ux*math.pi)/180
                uy = (uy*math.pi)/180

                position_point = Point()
                position_point.x = ux
                position_point.y = 0
                position_point.z = self.area

                self.position_pub.publish(position_point)

            self.find_ball.data = True
            self.ball_pub.publish(self.find_ball)

        else:

            if not self.noball_start_time:
                self.start_search =  time.time()
            if not self.noball_end_time:
                if (self.noball_now_time - self.noball_start_time) > 3:
                    self.noball_end_time = True
                
                self.noball_now_time = time.time()
            else:
                #self.noball_start_time = False
                #self.noball_end_time = False
                #self.noball_now_time = 0

                self.find_ball.data = False
                self.ball_pub.publish(self.find_ball)

                self.errx1 = 0
                self.erry1 = 0

                self.errx2 = 0
                self.erry2 = 0

                """position_point = Point()
                position_point.x = 0
                position_point.y = 0
                position_point.z = self.area

                self.position_pub.publish(position_point)"""

                if not self.start_search:
                    self.start_search =  time.time()
                if not self.end_search:
                    self.turn_search.data = False
                    self.turnNsearch_pub.publish(self.turn_search)
                    if (self.now - self.start_search) > 5:
                        self.end_search = True
                    
                    self.now = time.time()
                else:
                    self.turn_search.data = True
                    self.turnNsearch_pub.publish(self.turn_search)
                    time.sleep(2.0)
                    self.start_search = False
                    self.end_search = False
                    self.now = 0
                
                """self.t += 1

                if (self.t>360):
                    self.t=-360

                ux_noball = 70*math.sin(0.1*self.t) #(t/(180/3.14159))
                uy_noball = 20*math.sin(0.1*self.t) - 20

                ux_noball = (ux_noball*math.pi)/180
                uy_noball = (uy_noball*math.pi)/180

                position_point = Point()

                position_point.x = ux_noball
                position_point.y = uy_noball

                self.position_pub.publish(position_point)"""


if __name__ == "__main__":
    headctrl = HeadControl()
    while not rospy.is_shutdown():
        try:
            headctrl.control()
        except AttributeError:
            continue
