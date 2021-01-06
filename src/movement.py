import rospy
import os
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist
i = 0


def main():
    takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
    land = rospy.Publisher('/bebop/land', Empty, queue_size=1)
    move = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=100)
    empty_msg = Empty()
    move_msg = Twist()
    global i
    rospy.sleep(1)
    takeoff.publish(empty_msg)
    strength = 0.2
    rospy.sleep(3)
    move_msg.linear.x = strength
    move_msg.linear.y = 0
    move_msg.linear.z = 0
    move_msg.angular.z = 0
    for j in range (0,100):
        move.publish(move_msg)
    rospy.sleep(2)
    move_msg.linear.x = - strength
    move_msg.linear.y = 0
    move_msg.linear.z = 0
    move_msg.angular.z = 0
    for j in range (0,100):
        move.publish(move_msg)
    rospy.sleep(2)
    move_msg.linear.x = 0
    move_msg.linear.y = strength
    move_msg.linear.z = 0
    move_msg.angular.z = 0
    for j in range (0,100):
        move.publish(move_msg)
    rospy.sleep(2)
    move_msg.linear.x = 0
    move_msg.linear.y = - strength
    move_msg.linear.z = 0
    move_msg.angular.z = 0
    for j in range (0,100):
        move.publish(move_msg)
    rospy.sleep(2)
    move_msg.linear.x = 0
    move_msg.linear.y = 0
    move_msg.linear.z = strength
    move_msg.angular.z = 0
    for j in range (0,100):
        move.publish(move_msg)
    rospy.sleep(2)
    move_msg.linear.x = 0
    move_msg.linear.y = 0
    move_msg.linear.z = - strength
    move_msg.angular.z = 0
    for j in range (0,100):
        move.publish(move_msg)
    rospy.sleep(2)
    move_msg.linear.x = 0
    move_msg.linear.y = 0
    move_msg.linear.z = 0
    move_msg.angular.z = 0
    move.publish(move_msg)
    rospy.sleep(5)
    land.publish(empty_msg)
    rospy.spin()

if __name__ == '__main__':
	rospy.init_node("nawak")
    	main()
