#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class CarController:
    def __init__(self):
        rospy.init_node('car_controller', anonymous=True)
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.wheel_joints = ['front_left_wheel_joint', 'front_right_wheel_joint', 
                             'rear_left_wheel_joint', 'rear_right_wheel_joint']
        self.wheel_speed = 0.0

    def cmd_vel_callback(self, msg):
        # Map linear.x to wheel speed
        self.wheel_speed = msg.linear.x

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = self.wheel_joints
            joint_state.velocity = [self.wheel_speed] * 4  # Set speed for all wheels
            self.joint_state_pub.publish(joint_state)
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = CarController()
        controller.run()
    except rospy.ROSInterruptException:
        pass



# def controller():
#     rospy.init_node("controller")
#     pub = rospy.Publisher("/cmd_vel", Twist, queue_size= 10)
#     rate = rospy.Rate(10)
#     cmd = Twist()

#     while not rospy.is_shutdown():
#         key = input().lower()
#         if key == "w":
#             cmd.linear.x = 1
#         elif key == "s":
#             cmd.linear.x = -1
#         elif key == "a":
#             cmd.angular.z = 1
#         elif key == "d":
#             cmd.angular.z = -1
#         elif key == "q":
#             break
#         else:
#             cmd.linear.x = 0
#             cmd.angular.z = 0

#         pub.publish(cmd)
#         rate.sleep()

# if __name__ == "__main__":
#     try:
#         controller()
#     except rospy.ROSInterruptException:
#         pass

