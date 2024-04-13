#!/usr/bin/env python2.7

import rospy
from BallDetection import BallDetection as BD

if __name__ == "__main__":
    rospy.init_node('image')
    image_processor = BD()

    while not rospy.is_shutdown():
        try:
            image_processor.detect_ball()
        except AttributeError:
            continue
        