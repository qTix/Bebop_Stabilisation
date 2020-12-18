# Bebop_Stabilisation

[Installation Bebob](https://bebop-autonomy.readthedocs.io/en/latest/installation.html)  
[SIFT](https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf)  
[SURF](https://link.springer.com/chapter/10.1007/11744023_32)  
[Detection de contours](https://docs.opencv.org/master/df/d0d/tutorial_find_contours.html)  
[General bebop](https://bebop-autonomy.readthedocs.io/en/latest/index.html)  
[Topic bebop](https://bebop-autonomy.readthedocs.io/en/latest/reading.html) 

Build OpenCV:
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D OPENCV_ENABLE_NONFREE=ON

**Running the driver as a Node**
====
    $ roslaunch bebop_driver bebop_node.launch

**Takeoff**
====
    $ rostopic pub --once [namespace]/takeoff std_msgs/Empty

**Land**
====
    $ rostopic pub --once [namespace]/land std_msgs/Empty

**Emergency**
====
    $ rostopic pub --once [namespace]/reset std_msgs/Empty

**Pilot**
====
    linear.x  (+)      Translate forward
              (-)      Translate backward
    linear.y  (+)      Translate to left
              (-)      Translate to right
    linear.z  (+)      Ascend
              (-)      Descend
    angular.z (+)      Rotate counter clockwise
              (-)      Rotate clockwise
