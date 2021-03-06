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
import cv2, sys, time, collections

BUFFER_SIZE = 15

class images_motion(object):

    def __init__(self):
        self.proc_image_pub = rospy.Publisher('/workstation/proc_image', Image, queue_size = 1)
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.dev_pub = rospy.Publisher('/workstation/deviation', Vector3, queue_size = 5)
        self.move_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

        self.stab_sub = rospy.Subscriber("/bebop/stabilize", Empty, self.handle_stab, queue_size = 1)
        self.raw_imb_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback, queue_size = 1)

        self.deviation = Vector3()
        self.empty_msg = Empty()
        self.bridge = CvBridge()
        self.twist_msg = Twist()

        self.prev_gray = None
        self.transforms = []
        self.stabilize = False
        self.last_stab_time = 0
        self.dev_buffer_x = collections.deque(maxlen=BUFFER_SIZE)
        self.dev_buffer_y = collections.deque(maxlen=BUFFER_SIZE)
        self.dev_buffer_z = collections.deque(maxlen=BUFFER_SIZE)

    def callback(self, msg):
        msg_time = msg.header.stamp
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
                        self.deviation.y = self.transforms[-1][0]
                        self.deviation.z = self.transforms[-1][1]
                        self.deviation.x = self.transforms[-1][2]
                        #self.dev_history.append([msg_time, self.deviation])
                        # print(self.dev_history)
                        #threshhold_left_right = 0.5
                        #threshhold_up_down = 0.5
                        #self.dev_buffer_x.append(self.deviation.x)
                        #self.dev_buffer_y.append(self.deviation.y)
                        #self.dev_buffer_z.append(self.deviation.z)
                        #mean_y = sum(self.dev_buffer_y)/BUFFER_SIZE
                        #mean_z = sum(self.dev_buffer_z)/BUFFER_SIZE
                        decision = ""

                        if self.deviation.x > 1:
                            self.deviation.x = 1
                        if self.deviation.x < -1:
                            self.deviation.x = -1
                        if self.deviation.y >1:
                            self.deviation.y = 1
                        if self.deviation.y < -1:
                            self.deviation.y = -1
                        if self.deviation.z > 1:
                            self.deviation.z = 1
                        if self.deviation.z < -1:
                            self.deviation.z = -1

                        if 1 > self.deviation.y > 0.2:
                            self.twist_msg.linear.y =  - 0.5* self.deviation.y/2
                            decision = "go right :" +str(self.twist_msg.linear.y)
                            #self.twist_msg.linear.y = - mean_y
                        if -1 < self.deviation.y < -0.2:
                            self.twist_msg.linear.y =  - 0.5*self.deviation.y/2
                            decision = "go left :" +str(self.twist_msg.linear.y)
                            #self.twist_msg.linear.y = mean_y

                        if 1 > self.deviation.z > 0.5:
                            self.twist_msg.linear.z =  - 0.5*self.deviation.z/2
                            decision = "go down :"+str(self.twist_msg.linear.z)
                            #self.twist_msg.linear.z = -mean_z
                        if -1 < self.deviation.z < -0.5:
                            self.twist_msg.linear.z = - 0.5*self.deviation.z/2
                            decision = "go up :"+str(self.twist_msg.linear.z)
                            #self.twist_msg.linear.z = mean_z

                        if 1 > self.deviation.x > 0.5:
                            self.twist_msg.linear.x =  - 0.5*self.deviation.x/2
                            decision = "go forward :"+str(self.twist_msg.linear.x)
                            #self.twist_msg.linear.z = -mean_z
                        if -1 < self.deviation.x < -0.5:
                            self.twist_msg.linear.x = - 0.5*self.deviation.x/2
                            decision = "go backward :"+str(self.twist_msg.linear.x)
                            #self.twist_msg.linear.z = mean_z

                        print(decision)
                        self.twist_msg.angular.z = 0
                        self.twist_msg.linear.x = 0
                        self.twist_msg.linear.y = 0
                        self.twist_msg.linear.z = 0
                        self.move_pub.publish(self.twist_msg)


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
