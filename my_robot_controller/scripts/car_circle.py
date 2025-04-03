#!/usr/bin/env python3
import rospy
import numpy
# type: "geometry_msgs/Twist"
# imports Twist from the msg folder from the geometry_masgs package
from geometry_msgs.msg import Twist

import sys
import termios
import tty

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


if __name__ == "__main__":

    rospy.init_node("controller")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size= 10)
    rate = rospy.Rate(2)
    cmd = Twist()

    while not rospy.is_shutdown():
        key = get_key().lower()

        if key == "w":
            cmd.linear.x = 1
            cmd.angular.z = 0
        elif key == "s":
            cmd.linear.x = -1
            cmd.angular.z = 0
        elif key == "a":
            cmd.angular.z = 1
            cmd.linear.x = 0
        elif key == "d":
            cmd.angular.z = -1
            cmd.linear.x = 0
        elif key == "q":
            break
        elif key =="e":
            cmd.angular.z = 0
            cmd.linear.x = 0
        else:
            cmd.linear.x = 0
            cmd.angular.z = 0

        pub.publish(cmd)
        rate.sleep()



    # rospy.init_node("draw_circle")
    # rospy.loginfo("node has been started")

    # #tells you which topic you want to publish to
    # #in this case "command velocity" found through rostopic list
    # pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # rate = rospy.Rate(2)
    # sleep = rospy.Rate(1)

    # while not rospy.is_shutdown():
    #     # publish cmd_vel
    #     # innholdet av "brevet", som er Twist
    #     msg = Twist()

    #     msg.linear.x = 0.0
    #     msg.angular.z = 0.1

    #     # hvor du skal sende "brevet" til
    #     pub.publish(msg)

    #     #holder loope til 2hz
    #     rate.sleep()

    