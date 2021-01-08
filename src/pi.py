#! /usr/bin/python

import rospy
import numpy as np
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Twist
# OpenCV2 for saving an image
import cv2, sys, time, collections, csv

HISTORY_LEN = 15
Kp = 0.1
Ki = 0.2e-9
TRESHHOLD = 0.5

class images_motion(object):

    def __init__(self):
        # self.proc_image_pub = rospy.Publisher('/workstation/proc_image', Image, queue_size = 1)
        # self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        # self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        # self.dev_pub = rospy.Publisher('/workstation/deviation', Vector3, queue_size = 5)
        self.control_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)

        self.stab_sub = rospy.Subscriber("/bebop/stabilize", Empty, self.handle_stab, queue_size = 1)
        self.raw_imb_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback, queue_size = 1)

        self.deviation = Vector3()
        self.empty_msg = Empty()
        self.bridge = CvBridge()

        self.prev_gray = None
        self.transforms = []
        self.stabilize = False
        self.last_stab_time = 0

        self.dev_history = collections.deque(maxlen = HISTORY_LEN)

        self.file_odom = open("../data/odometry.csv", "wb")
        self.writer = csv.writer(self.file_odom)
        self.writer.writerow( ('Timestamp', 'x', 'y', 'z') )

    def callback(self, msg):
        msg_time = msg.header.stamp.secs + msg.header.stamp.nsecs*1e-09
        if self.stabilize:
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

                    if m is not None:
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

                        #print(str(np.round(self.transforms[-1],1)))
                        if -0.2 < self.deviation.y < 0.2:
                            self.deviation.y = self.transforms[-1][0]
                        else:
                            self.deviation.y = 0.0

                        if -0.2 < self.deviation.z < 0.2:
                            self.deviation.z = self.transforms[-1][1]
                        else:
                            self.deviation.z = 0.0
                        # print(self.dev_history)

                        # Initialize movement to 0:
                        last_stab_control = Twist()

                        # Proportial and integral reaction control
                        y_react_p = Kp * self.deviation.y
                        y_react_i = 0.0
                        z_react_p = Kp * self.deviation.z
                        z_react_i = 0.0
                        buffer_size = len(self.dev_history)
                        for i in range(buffer_size-1):
                            dt = self.dev_history[buffer_size-i-1][0] - self.dev_history[buffer_size-i-2][0]
                            y_react_i+= Ki * self.dev_history[buffer_size-i-1][1].y * dt
                            z_react_i+= Ki * self.dev_history[buffer_size-i-1][1].z * dt
                        y_react = y_react_i + y_react_p
                        z_react = z_react_i + z_react_p
                        # print("prop y \t" + str(np.round(y_react_p,3)))
                        # print("int y \t" + str(np.round(y_react_i,3)))

                        # print("z correct: " + str(z_react) + "\t y correct: " + str(y_react))
                        if y_react > TRESHHOLD:
                            y_react = TRESHHOLD
                        if y_react < -TRESHHOLD:
                            y_react = -TRESHHOLD
                        if z_react > TRESHHOLD:
                            z_react = TRESHHOLD
                        if z_react < -TRESHHOLD:
                            z_react = -TRESHHOLD
                        last_stab_control.linear.y = - y_react
                        last_stab_control.linear.z = z_react

                        self.control_pub.publish(last_stab_control)
                        print(last_stab_control.linear)

                    else:
                        print("Cannot find Rigid Transform")
        else:
            print("Stabilisation off")

    def handle_stab(self, msg):
        if time.time()-self.last_stab_time > 1:
            self.last_stab_time = time.time()
            self.stabilize = not self.stabilize
            if self.stabilize:
                print("Stabilisation on")
            else:
                print("Stabilisation off")




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


if __name__ == '__main__':
          main(sys.argv)
