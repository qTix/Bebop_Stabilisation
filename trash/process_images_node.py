#! /usr/bin/python

import rospy, cv2, sys, os, time
import numpy as np
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty

class process_images(object):

    def __init__(self):
        self.proc_image_pub = rospy.Publisher('/workstation/proc_image', Image, queue_size = 1)
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)

        self.raw_imb_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)

        self.empty_msg = Empty()
        self.bridge = CvBridge()
        self.folder_name = os.path.join("./",str(time.time()))
        os.mkdir(self.folder_name)

    def callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            time = msg.header.stamp
            gray = cv2.cvtColor(cv2_img,cv2.COLOR_BGR2GRAY)
            corners = cv2.goodFeaturesToTrack(gray, maxCorners=200,
                                                    qualityLevel=0.01,
                                                    minDistance=30,
                                                    blockSize=3)
            corners = np.int0(corners)
            for i in corners:
                x,y = i.ravel()
                cv2.circle(cv2_img,(x,y),6,255,-1)
            cv2.imwrite(os.path.join(self.folder_name,str(time)+'.jpeg'), cv2_img)

        try:
            self.proc_image_pub.publish(self.bridge.cv2_to_imgmsg(cv2_img, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def takeoff(self):
        print("Taking off...")
        rospy.sleep(0.5)
        self.takeoff_pub.publish(self.empty_msg)

    def abbort_mission(self):
        print("LANDING !! ABORT MISSION !! LANDING !!")
        rospy.sleep(0.5)
        self.land_pub.publish(self.empty_msg)

def main(args):
    pim = process_images()
    rospy.init_node('process_images_node', anonymous=True)
    rospy.sleep(1)
    #pim.takeoff()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        pim.abbort_mission()
    cv2.destroyAllWindows()

if __name__ == '__main__':
          main(sys.argv)
