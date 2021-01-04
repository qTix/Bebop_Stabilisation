#!/usr/bin/env python
import rospy
from std_msgs import Empty
from std_msgs import UInt8
from nav_msgs import Odometry
from sensor_msgs import Image
from geometry_msgs import Twist 
import sys, tty, termios
import numpy as np
import cv2
import cv2.cv as cv
import math
import struct

def callback(msg):
    print(msg)
def callback2(msg):
    print(msg)

if __name__ == '__main__':
    rospy.init_node('decollage', anonymous=True)
    odometry = rospy.Subscriber("odom", Odometry, callback)
    images_raw = rospy.Subscriber("image_raw", Image, callback2)
    pilot = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.Publisher("[namespace]/takeoff", Empty, queue_size=10).publish()
    #flip = rospy.Publisher("[namespace]/flip", UInt8, queue_size=10)
    #flip.publish(0)
    rospy.Publisher("[namespace]/land", Empty, queue_size=10).publish()
