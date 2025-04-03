#!/usr/bin/env python3
import rospy
if __name__ == "__main__":
    rospy.init_node("test_node")
    rospy.loginfo("test node has been started")

    rate = rospy.Rate(10)

    #basically en infinite loop inntil exit knapp er trykket
    while not rospy.is_shutdown():
        rospy.loginfo("hello")
        rate.sleep()


