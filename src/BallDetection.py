#!/usr/bin/env python2.7

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

class BallDetection(object):
    def __init__(self):
        self.robot_id = rospy.get_param('robot_id', 0)
        self.pub_final = rospy.Publisher('/robotis_' + str(self.robot_id) + '/ImgFinal', Image, queue_size=1)
        self.pub_center = rospy.Publisher('/robotis_' + str(self.robot_id) + '/BallCenter', Point, queue_size=1)
        #self.subimg = rospy.Subscriber("/robotis_" + str(self.robot_id) +"/usb_cam/image_raw", Image, self.image_callback)
        self.subimg = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        
        self.bridge = CvBridge()
        self.ball_center = Point()
        self.cascade_path = '/home/robotis/blenders_ws/src/soccer_pkg/data/cascade_manual.xml'
        self.frame = None


    def image_callback(self, img_msg):
        rospy.loginfo(img_msg.header)
        try:
            self.frame = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError:
            rospy.logerr("CvBridge Error")
        
        dimg = self.detect_ball(self.frame)
        print(self.ball_center)

        final_img = self.bridge.cv2_to_imgmsg(dimg, "rgb8")
        self.pub_final.publish(final_img)
        

    def detect_ball(self, frame):
        if frame is not None:
            pelota_clas = cv2.CascadeClassifier(self.cascade_path)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ball = pelota_clas.detectMultiScale(gray, scaleFactor=1.25, minNeighbors=7)

            if len(ball) > 0:
                x, y, w, h = ball[0]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cx = int((x + (x + w)) / 2)
                cy = int((y + (y + h)) / 2)
                self.ball_center.x = cx
                self.ball_center.y = cy
                cv2.circle(frame, (cx, cy), radius=5, color=(0, 255, 0), thickness=1)
                self.pub_center.publish(self.ball_center)
            else:
                self.ball_center.x = 999
                self.ball_center.y = 999
                self.pub_center.publish(self.ball_center)
            
            return frame
    
    def listener(self):
        rospy.spin()
    