import rospy
import os
from std_msgs.msg import Empty, String

i = 0


def main():
    takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
    land = rospy.Publisher('/bebop/land', Empty, queue_size=1)
    move = rospy.Publisher('/bebop/cmd_vel', Twist)
    empty_msg = Empty()
    move_msg = Twist()
    global i
    rospy.sleep(1)
    takeoff.publish(empty_msg)
    if(i == 0):
        move_msg.linear.x = 0.15
        move_msg.linear.y = 0
        move_msg.linear.z = 0
        move_msg.angular.z = 0
        move.publish(move_msg)
        rospy.sleep(2)
        move_msg.linear.x = 0
        move_msg.linear.y = 0
        move_msg.linear.z = 0
        move_msg.angular.z = 0
        move.publish(move_msg)
        rospy.sleep(5)
        i += 1
    if(i == 1):
        land.publish(empty_msg)
        i += 1
    rospy.spin()

if __name__ == '__main__':
    main()
