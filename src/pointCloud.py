#!/usr/bin/env python

import rospy

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
#import pyrealsense2 as rs
from matplotlib import pyplot as plt



'''
def pclCallback(pcl2):
  print(pcl2.data)
'''
bridge = CvBridge()

def color_callback(msg):
    # print("Received color image!")
    global color_img
    color_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    #print("saving color image")
    #print(color_img)

def depth_callback(depth_msg):
    global depthimg
    bridge = CvBridge()
    depth_img = bridge.imgmsg_to_cv2(depth_msg, "32FC1")
    cv_image_array = np.array(depth_img, dtype=np.dtype("f8"))
    cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
    # cv_image_resized = cv2.resize(cv_image_norm, interpolation=cv2.INTER_CUBIC)
    depthimg = depth_img
    
    #print("saving depth image")
    #print(depthimg)


def camera_info_callback(msg):
    global K
    global aligned_depth_image
    cam_info = msg.K
    K = np.array(
        [
            [cam_info[0], 0.0, cam_info[2]],
            [0.0, cam_info[4], cam_info[5]],
            [0.0, 0.0, 0.0],
        ]
    )
    global data 
    data = rospy.wait_for_message("/frame", Int32)
    print(data)
    di = o3d.geometry.Image( np.array(depthimg)/1000 )
    ci = o3d.geometry.Image( np.array(color_img) )
    print(di)
    print(ci)
    
    intrinsic = o3d.camera.PinholeCameraIntrinsic(msg.width, msg.height, cam_info[0], cam_info[4], cam_info[2], cam_info[5])
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(ci, di)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    R = pcd.get_rotation_matrix_from_xyz((0,np.pi , np.pi))
    pcd = pcd.rotate(R)
    o3d.visualization.draw_geometries([pcd])
    
    print("saving camera_parameters")
    #print(K)
    
    


rospy.init_node('pointCloud')
if __name__ == '__main__':
  color_topic = "/camera/color/image_raw"
  rospy.Subscriber(color_topic, Image, color_callback)
  rospy.sleep(0.1)
  depth_topic = "/camera/aligned_depth_to_color/image_raw"
  rospy.Subscriber(depth_topic, Image, depth_callback)
  rospy.sleep(0.1)
  cam_topic = "/camera/aligned_depth_to_color/camera_info"
  rospy.Subscriber(cam_topic, CameraInfo, camera_info_callback)
  
  
  print("hi")
  rospy.sleep(1)
  try:
   print(data)
  except:
   print("not foun")
  #rospy.sleep(10)
  rospy.spin()
  #print(pcd)
  #print(msg.data)
