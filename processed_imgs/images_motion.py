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
        self.proc_image_pub = rospy.Publisher('/workstation/proc_image', Image, queue_size = 1)
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)

        self.raw_imb_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback, queue_size = 1)

        self.empty_msg = Empty()
        self.bridge = CvBridge()

        self.prev_gray = None
        self.transforms = []

    def callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            curr_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)

            if self.prev_gray is None:
                curr_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
                self.prev_gray = curr_gray
            else:
                prev_pts = cv2.goodFeaturesToTrack(self.prev_gray,
                                                 maxCorners=200,
                                                 qualityLevel=0.01,
                                                 minDistance=30,
                                                 blockSize=3)

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

                if m:
                    # Extract traslation
                    dx = m[0,2]
                    dy = m[1,2]

                    # Extract rotation angle
                    da = np.arctan2(m[1,0], m[0,0])

                    # Store transformation
                    self.transforms.append([dx,dy,da])

                    # Move to next frame
                    self.prev_gray = curr_gray

                    # print("Tracked points : " + str(len(prev_pts)))
                    # print(np.round(self.transforms[-1],1))
                    # Compute trajectory using cumulative sum of transformations
                    trajectory = np.cumsum(transforms, axis=0)
                    smoothed_trajectory = smooth(trajectory)

                    # Calculate difference in smoothed_trajectory and trajectory
                    difference = smoothed_trajectory - trajectory

                    # Calculate newer transformation array
                    transforms_smooth = transforms + difference
                    # Extract transformations from the new transformation array
                    dx = transforms_smooth[i,0]
                    dy = transforms_smooth[i,1]
                    da = transforms_smooth[i,2]
                    # Reconstruct transformation matrix accordingly to new values
                    m = np.zeros((2,3), np.float32)
                    m[0,0] = np.cos(da)
                    m[0,1] = -np.sin(da)
                    m[1,0] = np.sin(da)
                    m[1,1] = np.cos(da)
                    m[0,2] = dx
                    m[1,2] = dy

                    print(m)

                else:
                    print("Cannot find Rigid Transform")


def main(args):
    pim = images_motion()
    rospy.init_node('process_images_node', anonymous=True)
    rospy.Rate(30)
    #pim.takeoff()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        pim.abbort_mission()
    cv2.destroyAllWindows()

def movingAverage(curve, radius):
  window_size = 2 * radius + 1
  # Define the filter
  f = np.ones(window_size)/window_size
  # Add padding to the boundaries
  curve_pad = np.lib.pad(curve, (radius, radius), 'edge')
  # Apply convolution
  curve_smoothed = np.convolve(curve_pad, f, mode='same')
  # Remove padding
  curve_smoothed = curve_smoothed[radius:-radius]
  # return smoothed curve
  return curve_smoothed

def smooth(trajectory):
  smoothed_trajectory = np.copy(trajectory)
  # Filter the x, y and angle curves
  for i in range(3):
    smoothed_trajectory[:,i] = movingAverage(trajectory[:,i], radius=SMOOTHING_RADIUS)

  return smoothed_trajectory

if __name__ == '__main__':
          main(sys.argv)
