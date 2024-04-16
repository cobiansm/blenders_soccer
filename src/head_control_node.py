#!/usr/bin/env python2.7
#coding=utf-8

import rospy
from HeadControl import HeadControl as HC

def check_player(obj):
    if obj.player not in obj._players_list:
        raise ValueError("Invalid Type Player")
    
if __name__ == "__main__":
    rospy.init_node('head_control_node')
    headctrl = HC()
    headctrl.player = "Defender" #rospy.get_param("player", 0)

    if headctrl.player == "Goalkeeper":
        from BallPrediction import BallPrediction as BP; bp = BP()
    
    try:
        check_player(headctrl)
    except ValueError as e:
        print(e)
    
    while not rospy.is_shutdown():
        if headctrl.player == "Goalkeeper":
            bp.get_predictions(frame=headctrl.frame, center=headctrl.center, predict_num=4)
        #try:
        headctrl.head_control()
        # except AttributeError:
        #     continue
