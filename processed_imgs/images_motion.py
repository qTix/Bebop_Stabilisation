#! /usr/bin/python

import rospy
import numpy as np
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty
# OpenCV2 for saving an image
import cv2
import sys

class images_motion(object):

    def __init__(self):
        self.proc_image_pub = rospy.Publisher('/workstation/proc_image', Image, queue_size = 10)
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)

        self.raw_imb_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)

        self.empty_msg = Empty()
        self.bridge = CvBridge()

    def callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            curr_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
            
            if not self.prev_gray:
                curr_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
                self.prev_gray = curr_gray


            prev_pts = cv2.goodFeaturesToTrack(self.prev_gray,20,0.50,20,3)

            # Convert to grayscale
            gray = cv2.cvtColor(cv2_img,cv2.COLOR_BGR2GRAY)

            # Calculate optical flow (i.e. track feature points)
            curr_pts, status, err = cv2.calcOpticalFlowPyrLK(self.prev_gray, curr_gray, prev_pts, None)

            # Filter only valid points
            idx = np.where(status==1)[0]
            prev_pts = prev_pts[idx]
            curr_pts = curr_pts[idx]

  #Find transformation matrix
  m = cv2.estimateRigidTransform(prev_pts, curr_pts, fullAffine=False) #will only work with OpenCV-3 or less

  # Extract traslation
  dx = m[0,2]
  dy = m[1,2]

  # Extract rotation angle
  da = np.arctan2(m[1,0], m[0,0])

  # Store transformation
  transforms[i] = [dx,dy,da]

  # Move to next frame
  prev_gray = curr_gray

  print("Frame: " + str(i) +  "/" + str(n_frames) + " -  Tracked points : " + str(len(prev_pts)))

            self.prev_gray = curr_gray

            # Visualisation of good goodFeaturesToTrack
            # for i in self.prev_corners:
            #     x,y = i.ravel()
            #     cv2.circle(cv2_img,(x,y),6,255,-1)
            # time = msg.header.stamp
            #cv2.imwrite(os.path.join(folder_name,str(time)+'.jpeg'), cv2_img)

        try:
            self.proc_image_pub.publish(self.bridge.cv2_to_imgmsg(cv2_img, "bgr8"))
        except CvBridgeError as e:
            print(e)
        rospy.sleep(0.1)

    def takeoff(self):
        print("Taking off...")
        rospy.sleep(0.5)
        self.takeoff_pub.publish(self.empty_msg)

    def abbort_mission(self):
        print("LANDING !! ABORT MISSION !! LANDING !!")
        rospy.sleep(0.5)
        self.land_pub.publish(self.empty_msg)

def main(args):
    pim = images_motion()
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
