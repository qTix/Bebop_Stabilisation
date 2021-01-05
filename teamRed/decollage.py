#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist 
import sys, tty, termios
import numpy as np
import cv2 as cv
import math
import struct

def callback(msg):
    print("Odometry: "+msg)
def callback2(msg):
    print("Image_raw: "+msg)

if __name__ == '__main__':
    rospy.init_node('decollage', anonymous=True)
    odometry = rospy.Subscriber("odom", Odometry, callback)
    images_raw = rospy.Subscriber("image_raw", Image, callback2)
    #pilot = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    #rospy.Publisher("[namespace]/takeoff", Empty, queue_size=10).publish()
    #flip = rospy.Publisher("[namespace]/flip", UInt8, queue_size=10)
    #flip.publish(0)
    #rospy.Publisher("[namespace]/land", Empty, queue_size=10).publish()
    while not rospy.is_shutdown():
        continue

