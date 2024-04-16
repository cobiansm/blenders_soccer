#!/usr/bin/env python2.7

import rospy
import sensor_msgs
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
import math
import std_msgs
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import os

def angleCallback(angle_msg):
    global half_boolean
    rospy.loginfo(angle_msg.position)
    angle_msg=angle_msg.position[1]
    if angle_msg<-0.7:
        half_boolean=True
    else:
        half_boolean=False

def imageCallback(img_msg):
    rospy.loginfo(img_msg.header)

    try:
        frame = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        # cv_img = cv2.flip(cv_img,1)
    except CvBridgeError:
        rospy.logerr("CvBridge Error")
    # continue

    img, center = ball_detect(frame)  ##como le meto mi imagen y saco lo de cv2
    img = get_predictions(img, center)

    final_img = bridge.cv2_to_imgmsg(img, "rgb8")
    # mask_img = bridge.cv2_to_imgmsg(dummy_mask)
    # frame_img = bridge.cv2_to_imgmsg(frame, "rgb8") # Si es imagen binaria, quitar "rgb8"

    pub_final.publish(final_img)
    # pub_mask.publish(mask_img)
    # pub_frame.publish(frame_img)


# cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
def ball_detect(frame):
    global center, half_boolean

    pelota_clas = cv2.CascadeClassifier('/home/robotis/blenders_ws/src/soccer_pkg/data/cascade_manual.xml')
    half_clas = cv2.CascadeClassifier('/home/robotis/blenders_ws/src/soccer_pkg/data/half_cascade.xml')
    

    
    # ret,frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if half_boolean:

        ball = half_clas.detectMultiScale(gray,
                                           scaleFactor=1.25,
                                           minNeighbors=5)
    else:
        ball = pelota_clas.detectMultiScale(gray,
                                           scaleFactor=1.25,
                                           minNeighbors=10, minSize=(50, 50))

    if len(ball)>0:
        x,y,w,h = ball[0]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cx = int((x + (x + w)) / 2)
        cy = int((y + (y + h)) / 2)
        center.x = cx
        center.y = cy
        cv2.circle(frame, (cx, cy), radius=5, color=(0, 255, 0), thickness=1)
        pub_center.publish(center)
    else:
        center.x = 999
        center.y = 999
        pub_center.publish(center)

    return frame, center

def get_predictions(frame, center, predict_num=4):
    for p in range(predict_num):
        if p == 0:
            prediction = _predict(np.array((center.x, center.y), np.float32))
            first_prediction = prediction
        else:
            prediction = _predict(np.array((old_prediction[0], old_prediction[1]), np.float32))
        cv2.circle(frame, (prediction[0], prediction[1]),5, color=(255,0,0), thickness=3)
        old_prediction = prediction
    
    try:
        slope = (prediction[1] - first_prediction[1]) / (prediction[0] - first_prediction[0])

        # calculate y-intercept
        b = first_prediction[1]- (slope * first_prediction[0])

        # draw infinite line
        cv2.line(frame, (0, int(b)), (1280, int((slope * 1280) + b)), color=(255,255,255), thickness=2)
        cv2.line(frame, (first_prediction[0], first_prediction[1]), (first_prediction[0],prediction[1]), color=(255,255,255), thickness=2)
    except:
            pass
    
    return frame

def _predict(value):
    global kalman
    kalman.correct(value)
    tp = kalman.predict()
    return int(tp[0]), int(tp[1])


if __name__ == "__main__":
    half_boolean = None


    n_states = 4
    n_measures = 2
    kalman = cv2.KalmanFilter(n_states, n_measures)
    kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    kalman.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) * 0.03


    rospy.init_node('image')
    robot_id = rospy.get_param('robot_id', 0)
    pub_final = rospy.Publisher('/robotis_' + str(robot_id) + '/ImgFinal', Image, queue_size=1)
    pub_center = rospy.Publisher('/robotis_' + str(robot_id) + '/BallCenter', Point, queue_size=1)
    #self.subimg = rospy.Subscriber("/robotis_" + str(self.robot_id) +"/usb_cam/image_raw", Image, self.image_callback)

    rospy.loginfo("Hello ros")

    bridge = CvBridge()

    subimg = rospy.Subscriber("/usb_cam/image_raw", Image, imageCallback)
    #subscriptor del angulo de la cabeza
    subangle_head = rospy.Subscriber("robotis_0/present_joint_states",Float64, angleCallback)

    center = Point()

    while not rospy.is_shutdown():
        pass
