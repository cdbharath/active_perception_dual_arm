#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
import numpy as np

bridge = CvBridge()

def image_publisher_cb(data):
    # alpha = 1
    # beta = 0
    
    np_arr = np.fromstring(data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    # brightness_corrected = np.clip(alpha*image_np + beta, 0, 255)    
    
    ycrcb_img = cv2.cvtColor(image_np, cv2.COLOR_BGR2YCrCb)
    # equalize the histogram of the Y channel
    ycrcb_img[:, :, 0] = cv2.equalizeHist(ycrcb_img[:, :, 0])
    # convert back to RGB color-space from YCrCb
    equalized_img = cv2.cvtColor(ycrcb_img, cv2.COLOR_YCrCb2BGR)
    brightness_corrected = equalized_img
    
    image_message = bridge.cv2_to_imgmsg(brightness_corrected, encoding="bgr8")
    image_pub.publish(image_message)
    
if __name__ == "__main__":
    rospy.init_node("brightness_correction")

    rospy.Subscriber("main_cam/color/image_raw/compressed", CompressedImage, image_publisher_cb)
    image_pub = rospy.Publisher("main_cam/brightness_corrected", Image, queue_size=10)
    rospy.spin()