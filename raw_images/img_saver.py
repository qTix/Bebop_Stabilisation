#! /usr/bin/python

import rospy
import time
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty
# OpenCV2 for saving an image
import cv2
import os

# Instantiate CvBridge
bridge = CvBridge()
folder_name = os.path.join("./",str(time.time()))

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        time = msg.header.stamp
        print(os.path.join(folder_name,str(time)+'.jpeg'))
        cv2.imwrite(os.path.join(folder_name,str(time)+'.jpeg'), cv2_img)
        rospy.sleep(0.1)

def main():
    rospy.init_node('image_listener')
    os.mkdir(folder_name)
    # Define your image topic
    image_topic = "/bebop/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)

    takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
    land = rospy.Publisher('/bebop/land', Empty, queue_size=1)
    empty_msg = Empty()

    rospy.sleep(1)
    takeoff.publish(empty_msg)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
