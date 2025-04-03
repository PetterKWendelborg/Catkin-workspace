#!/usr/bin/env python3
import rospy
import numpy
# type: "geometry_msgs/Twist"
# imports Twist from the msg folder from the geometry_masgs package
from geometry_msgs.msg import Twist



if __name__ == "__main__":
    rospy.init_node("draw_circle")
    rospy.loginfo("node has been started")

    #tells you which topic you want to publish to
    #in this case "command velocity" found through rostopic list
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(2)
    sleep = rospy.Rate(1)

    while not rospy.is_shutdown():
        # publish cmd_vel
        # innholdet av "brevet", som er Twist
        msg = Twist()

        msg.linear.x = 2.0
        msg.angular.z = 1.0

        # hvor du skal sende "brevet" til
        pub.publish(msg)

        #holder loope til 2hz
        rate.sleep()



