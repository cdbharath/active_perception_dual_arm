#!/usr/bin/env python

from kinova_ik.kinova_gen3_ik import KinovaGen3IK
from math import pi

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from tf.transformations import quaternion_from_euler

class IKNode:
    def __init__(self):
        self.kinova_ik = KinovaGen3IK()

        self.joint1_pub = rospy.Publisher("j1_pos_cmd", Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher("j2_pos_cmd", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("j3_pos_cmd", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("j4_pos_cmd", Float64, queue_size=10)
        self.joint5_pub = rospy.Publisher("j5_pos_cmd", Float64, queue_size=10)
        self.joint6_pub = rospy.Publisher("j6_pos_cmd", Float64, queue_size=10)
        self.joint7_pub = rospy.Publisher("j7_pos_cmd", Float64, queue_size=10)

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

        # success, joints = self.kinova_ik.solve_ik([0.5, 0, 0.5], 
        #                                  quaternion_from_euler(0, 1.57, 0).tolist())

        self.joint1_pub.publish(joints[0])
        self.joint2_pub.publish(joints[1])
        self.joint3_pub.publish(joints[2])
        self.joint4_pub.publish(joints[3])
        self.joint5_pub.publish(joints[4])
        self.joint6_pub.publish(joints[5])
        self.joint7_pub.publish(joints[6])


        rospy.loginfo("Published joint commands %s", success)
        rospy.loginfo([x, y, z, R, P, Y])
        rospy.loginfo(self.kinova_ik.get_link_pose())

if __name__ == "__main__":
    rospy.init_node("kinova_ik")

    IKNode()

    # kinova_ik = KinovaGen3IK()
    # joints = kinova_ik.solve_ik([0.0, 0.0, 0.50], 
    #                         quaternion_from_euler(0, pi/2, 0).tolist())
    # print(joints) #zyx

    # kinova_ik.visualize()

    # angle, pose = kinova_ik.forward_kinematics(joints)
    # print(pose, angle)

    # print(kinova_ik.get_link_pose())

    rospy.spin()