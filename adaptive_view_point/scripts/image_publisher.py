#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
import numpy as np

bridge = CvBridge()

def image_publisher_cb(timer):
    cv_image = cv2.imread("aruco.png")
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    image_pub.publish(image_message)

def compressed_image_publisher_cb(timer):
    image_np = cv2.imread("aruco.png")
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()  
    image_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("image_publisher")
    image_pub = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=10)
    image_pub = rospy.Publisher("camera/color/image_raw/compressed", CompressedImage, queue_size=10)
    rospy.Timer(rospy.Duration(0.1), compressed_image_publisher_cb)
    rospy.Timer(rospy.Duration(0.1), image_publisher_cb)
    # rospy.logwarn("Timer Initiated")
    rospy.spin()