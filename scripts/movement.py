import rospy
import os
from std_msgs.msg import Empty, String

i = 0


def main():
    takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
    land = rospy.Publisher('/bebop/land', Empty, queue_size=1)
    move = rospy.Publisher('/bebop/cmd_vel', Twist)
    empty_msg = Empty()
    global i
    rospy.sleep(1)
    takeoff.publish(empty_msg)
    if(i == 0):
        move.publish('[0.15, 0.0, 0.0]')
        rospy.sleep(5)
        move.publish('[0.0, 0.0, 0.0]')
        i += 1
    if(i == 1):
        land.publish(empty_msg)
        i += 1
    rospy.spin()

if __name__ == '__main__':
    main()
