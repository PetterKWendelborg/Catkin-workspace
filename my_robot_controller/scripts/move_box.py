#!/usr/bin/env python3
import rospy
import tf
import tf.transformations
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist



def twist_callback(msg):
    global x, y, yaw

    linear_velocity = msg.linear.x
    angular_velocity = msg.angular.z

    x += linear_velocity *0.1
    yaw += angular_velocity *0.1


    transform = geometry_msgs.msg.TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id ="world"
    transform.child_frame_id = "base_link"

    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = 1.0

    quaternion = tf.transformations.quaternion_from_euler(0,0,yaw)
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]

    tf_broadcaster.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node("twist_to_tf")
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    x,y,yaw = 0.0, 0.0 ,0.0

    rospy.Subscriber("/cmd_vel", Twist, twist_callback)

    rospy.spin()



