#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2

def color_callback(msg):
    global color_img
    color_img = bridge.imgmsg_to_cv2(msg, "bgr8")

def depth_callback(depth_msg):
    global depth_img
    depth_img = bridge.imgmsg_to_cv2(depth_msg, "32FC1")

def writer():
    cv2.imwrite('rgbd/rgb.jpg', color_img)
    cv2.imwrite('rgbd/depth.png', depth_img)

bridge = CvBridge()  
rospy.init_node('img_getter')

if __name__ == '__main__':
  color_topic = "/camera/color/image_raw"
  rospy.Subscriber(color_topic, Image, color_callback)
  rospy.sleep(0.1)
  depth_topic = "/camera/aligned_depth_to_color/image_raw"
  rospy.Subscriber(depth_topic, Image, depth_callback)
  rospy.sleep(0.1)
  writer()
  rospy.sleep(1)
  rospy.spin()