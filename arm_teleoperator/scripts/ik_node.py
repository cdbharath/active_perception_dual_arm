#!/usr/bin/env python

from kinova_ik.kinova_gen3_ik import KinovaGen3IK
import rospy
from math import pi

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from tf.transformations import quaternion_from_euler

class IKNode:
    def __init__(self):
        self.kinova_ik = KinovaGen3IK()

        self.joint1_pub = rospy.Publisher("/left_arm_joint_1_position_controller/command", Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher("/left_arm_joint_2_position_controller/command", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("/left_arm_joint_3_position_controller/command", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("/left_arm_joint_4_position_controller/command", Float64, queue_size=10)
        self.joint5_pub = rospy.Publisher("/left_arm_joint_5_position_controller/command", Float64, queue_size=10)
        self.joint6_pub = rospy.Publisher("/left_arm_joint_6_position_controller/command", Float64, queue_size=10)
        self.joint7_pub = rospy.Publisher("/left_arm_joint_7_position_controller/command", Float64, queue_size=10)

        rospy.Subscriber("eef/pose", TwistStamped, self.ik_cb)

    def ik_cb(self, data):
        x = data.twist.linear.x
        y = data.twist.linear.y
        z = data.twist.linear.z

        R = data.twist.angular.x
        P = data.twist.angular.y
        Y = data.twist.angular.z

        success, joints = self.kinova_ik.solve_ik([x, y, z], 
                                         quaternion_from_euler(R, P, Y).tolist())

        self.joint1_pub.publish(joints[0]*180/pi)
        self.joint2_pub.publish(joints[1]*180/pi)
        self.joint3_pub.publish(joints[2]*180/pi)
        self.joint4_pub.publish(joints[3]*180/pi)
        self.joint5_pub.publish(joints[4]*180/pi)
        self.joint6_pub.publish(joints[5]*180/pi)
        self.joint7_pub.publish(joints[6]*180/pi)

        rospy.loginfo("Published joint commands %s", success)
        rospy.loginfo(self.kinova_ik.get_link_pose())

if __name__ == "__main__":
    rospy.init_node("kinova_ik")

    IKNode()

    # kinova_ik = KinovaGen3IK()
    # print(kinova_ik.solve_ik([0.5, 0.0, 0.00], 
    #                         quaternion_from_euler(pi/2, 0, pi/2).tolist())) #zyx

    # kinova_ik.visualize()
    # print(kinova_ik.get_link_pose())

    rospy.spin()