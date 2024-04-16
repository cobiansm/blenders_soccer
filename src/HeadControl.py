#!/usr/bin/env python2.7
#coding=utf-8

import time
import math
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from BallDetection import BallDetection

class HeadControl():
    def __init__(self):
        #super(HeadControl, self).__init__()
        self.robot_id = rospy.get_param('robot_id', 0)
        self.error_pub = rospy.Publisher('/robotis_' + str(self.robot_id) + '/error', Point, queue_size=1)
        self.position_pub = rospy.Publisher('/robotis_' + str(self.robot_id) + '/position', Point, queue_size=1)
        self.search_ball_pub = rospy.Publisher('/robotis_' + str(self.robot_id) + '/search_ball', Bool, queue_size=1)
        self.ball_pub = rospy.Publisher('/robotis_' + str(self.robot_id) + '/find_ball', Bool, queue_size=1)
        self.turnNsearch_pub = rospy.Publisher('/robotis_' + str(self.robot_id) + '/turnNsearch', Bool, queue_size=1)

        sub_center = rospy.Subscriber('/robotis_' + str(self.robot_id) + '/BallCenter', Point, self.callbackCenter)

        self.find_ball = Bool()
        self.search_ball = Bool()
        self.turn_search = Bool()
        self.center = Point()

        self.search_ball.data = False
        self.find_ball.data = False

        self.ux0 = 0
        self.ux0 = 0

        self.ux1 = 0
        self.uy1 = 0

        self.kpx = 0.00018
        self.kpy = 0.00018

        self.errx0 = 0
        self.errx1 = 0
        self.erry0 = 0
        self.erry1 = 0

        self.now = 0
        self.start_search = 0
        self.end_search = False

        self.noball_start_time = 0
        self.noball_now_time = 0
        self.noball_end_time = False
        self.safety_time = 3  #segs

        self.t = 0
        self.head_direction = 1

        self.player = None

        self._players_list = ["Goalkeeper","Defender","Midfielder","Forward"]
    

    def callbackCenter(self, ball_center):
        self.center.x = ball_center.x
        self.center.y = ball_center.y
    
    
    def head_control(self):
        self.turn_search.data = False
        self.turnNsearch_pub.publish(self.turn_search)

        point = self._calc_err()
        #print(self.center)
        #print(point)
        self.find_ball.data = self._is_ball_in_img()
        #print(point is None, self.find_ball.data)
        if point is None:
            #print("no point")
            self._search_for_ball()
        elif self.find_ball.data:
            #print("center ball")
            self._center_ball(point)


    def _calc_err(self):
        x_center = 320
        y_center = 240
        print(self.center)

        if self.center.x != 999 and self.center.y != 999 and self.center.x != None and self.center.y != None:
            errorx = x_center - self.center.x
            errory = y_center - self.center.y

            errorx = errorx * 70/x_center
            errory = errory * 70/y_center

            point = Point()
            point.x = errorx
            point.y = errory

            self.error_pub.publish(point)

            return point
        else:
            return None


    def _is_ball_in_img(self):
        # return (
        #     (self.errx0 < -35 or self.errx0 > 35)
        #     or (self.erry0 < -35 or self.erry0 > 35)
        #     or point is None
        # )
        return (
            (self.errx0 > -35 and self.errx0 < 35)
            or (self.erry0 > -35 and self.erry0 > 35)
        )
    

    def _search_for_ball(self):
        if not self.noball_start_time:
                self.noball_start_time =  time.time()
        if not self.noball_end_time:
            self.noball_end_time = (self.noball_now_time - self.noball_start_time) > self.safety_time
            self.noball_now_time = time.time()
        else:
            self.noball_start_time = False
            self.noball_end_time = False
            self.noball_now_time = 0

            ##self.find_ball.data = False
            ##self.ball_pub.publish(self.find_ball)
        
            self.ux1 = 0
            self.uy1 = 0

            self.search_ball.data = True
            self.search_ball_pub.publish(self.search_ball)

            self.t += 1 * self.head_direction

            if (self.t >= 360):
                self.head_direction = -1
            elif (self <= -360):
                self.head_direction = 1

            ux_noball = 60*math.sin(1*self.t) #(t/(180/3.14159))
            uy_noball = 15*math.cos(1*-self.t*self.head_direction) - 30

            ux_noball = (ux_noball*math.pi)/180
            uy_noball = (uy_noball*math.pi)/180

            position_point = Point()

            position_point.x = ux_noball
            position_point.y = uy_noball

            self.position_pub.publish(position_point)

            #time.sleep(0.05)

            self.errx1 = 0
            self.erry1 = 0

            if self.player != "Goalkeeper":
                self._check_env()
                
    
    ########Move this function to another file if it works
    def _check_env(self):
        if not self.start_search:
            self.start_search =  time.time()
        if not self.end_search:
            self.turn_search.data = False
            self.turnNsearch_pub.publish(self.turn_search)
            if (self.now - self.start_search) > 15:
                self.end_search = True
            
            self.now = time.time()
        else:
            self.turn_search.data = True
            self.turnNsearch_pub.publish(self.turn_search)
            time.sleep(1.0)
            self.start_search = False
            self.end_search = False
            self.now = 0
    
    
    def _center_ball(self, point):
        self.search_ball.data = False
        self.search_ball_pub.publish(self.search_ball)
        
        self.noball_start_time = 0
        self.noball_now_time = 0
        self.noball_end_time = False

        self.errx0 = point.x
        self.erry0 = point.y

        self.start_search = 0

        ux = self.ux1 + self.kpx*self.errx0 #- self.kp*self.errx1
        uy = self.uy1 + self.kpy*self.erry0 #- self.kp*self.erry1

        if ux >= 70: ux = 70
        elif ux <= -70: ux = -70

        if uy >= 20: uy = 20
        elif uy <= -70: uy = -70

        self.ux1 = ux
        self.errx1 = self.errx0

        self.uy1 = uy
        self.erry2 = self.erry1

        if (self.errx0 < -16 or self.errx0 > 16) or (self.erry0 < -16 or self.erry0 > 16):

            ux = (ux*math.pi)/180
            uy = (uy*math.pi)/180

            position_point = Point()
            position_point.x = ux
            position_point.y = uy

            self.position_pub.publish(position_point)

            self.find_ball.data = False
            self.ball_pub.publish(self.find_ball)
        else:
            self.find_ball.data = True
            self.ball_pub.publish(self.find_ball)

        self.center.x = None
        self.center.y = None
