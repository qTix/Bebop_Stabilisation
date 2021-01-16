# Bebop_Stabilisation

This repository is the working space of two project groups (Drone1 and Drone2) in the context of Robotic and Embedded Project (PRJREB), INSA Lyon.  
Each group has a working directory (Drone1/ and Drone2/), shared ressources areto be found in the main directory.

[Installation Bebob](https://bebop-autonomy.readthedocs.io/en/latest/installation.html)  
[SIFT](https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf)  
[SURF](https://link.springer.com/chapter/10.1007/11744023_32)  
[Detection de contours](https://docs.opencv.org/master/df/d0d/tutorial_find_contours.html)  
[General bebop](https://bebop-autonomy.readthedocs.io/en/latest/index.html)  
[Topic bebop](https://bebop-autonomy.readthedocs.io/en/latest/reading.html) 

Build OpenCV:
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D OPENCV_ENABLE_NONFREE=ON
    
# Repository composition
### src:
contains ROS source code with:  
 - **gftt.py**: a test file for the gftt function
 - **images_motion.py**: file for proportional reaction to derivation, with xbox controler switch for activate/deactivate the stabilisation process
 - **movement.py**: a test file to control the drone with a given sequence
 - **pi.py**: improvement of images_motion.py with proportional AND integral reaction to derivation, with xbox controler switch handling too
 - **stab.py**: proportional reaction prototype
 - **stab_image.py**: mean based reaction to derivation
 
 ### scripts:
 contains development scripts:
 - **driver.sh**: launches bebop autonomy driver
 - **plot.sh**: launches ROS ploting node on a given topic for real time data analysis
 
 ### launch:
 contains launch file for the joy_node, handling xbox controler inputs
 
 ### config:
 contains config file to handle xbox controler with xbox button being mapped to "activate/deactivate stabilisation process"


# ROS commands

Running the driver as a Node
----
    $ roslaunch bebop_driver bebop_node.launch

Takeoff
----
    $ rostopic pub --once [namespace]/takeoff std_msgs/Empty

Land
----
    $ rostopic pub --once [namespace]/land std_msgs/Empty

Emergency
----
    $ rostopic pub --once [namespace]/reset std_msgs/Empty

Pilot
----
    linear.x  (+)      Translate forward
              (-)      Translate backward
    linear.y  (+)      Translate to left
              (-)      Translate to right
    linear.z  (+)      Ascend
              (-)      Descend
    angular.z (+)      Rotate counter clockwise
              (-)      Rotate clockwise

# Video processing

(from https://www.learnopencv.com/video-stabilization-using-point-feature-matching-in-opencv/)

Import video, read frames
----

Video stream is published on `image_raw` topic as `sensor_msgs/Image` messages. (640 x 368 @ 30 Hz) ([doc](https://bebop-autonomy.readthedocs.io/en/latest/reading.html))  
We receive one image after another, so we'll need to store at least one opencv image in a 'buffer'.  
We need to convert this 'ROS styled' images flow to OpenCV format. We'll use ROS' package [vision_opencv](https://wiki.ros.org/vision_opencv).  

Using vision_opencv
----

[Tutorial](https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython) on converting ROS images to OpenCV format (Python).

    from cv_bridge import CvBridge
    bridge = CvBridge()
    cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')

We will need to pay attention to the encoding that we want, most common encoding for opencv is `bgr8`.

Convert to greyscales
----
Because we don't really need colors

    prev_gray = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY) 

Detect feature points
----

This method retrieve interesting points to track in the image, ie corners, straight lines, etc. You can change the number of points wanted, minimum space between them, etc.

- cv2.goodFeaturesToTrack() ([doc](https://docs.opencv.org/4.5.0/dd/d1a/group__imgproc__feature.html#ga1d6bb77486c8f92d79c8793ad995d541)))

Calculate optical flow
----

This method try to track each given feature in the next frame, you have to clean the results because errors may appear.

- cv2.calcOpticalFlowPyrLK() ([doc](https://docs.opencv.org/4.5.0/dc/d6b/group__video__track.html#ga473e4b886d0bcc6b65831eb88ed93323))

Estimate Motion
----

This methods uses the points of interest from the first frame, and the linked points of interest from the second frame to compute the transformation matrix.

**ATTENTION : this method is deprecated since OpenCV-4, see [cv2.estimateAffine2D()](https://docs.opencv.org/4.5.0/d9/d0c/group__calib3d.html#ga27865b1d26bac9ce91efaee83e94d4dd) and [cv2.estimateAffinePartial2D()](https://docs.opencv.org/4.5.0/d9/d0c/group__calib3d.html#gad767faff73e9cbd8b9d92b955b50062d) instead.**

- cv2.estimateRigidTransform() ([doc](https://docs.opencv.org/4.5.0/dc/d6b/group__video__track.html#ga762cbe5efd52cf078950196f3c616d48))
