#!/usr/bin/env python3

# General Imports
import cv2
import open3d as o3d

# ROS Imports 
import rospy
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo

# Own Imports
from franka_move import MoveGroupPyInterface
from rgbd_reconstruct import ReconstructionSystem
from tripoSR import LRM_Reconstruction

# ------------- ------------- Main ------------- -------------

rospy.init_node("eval_runner", anonymous=True)

move = MoveGroupPyInterface()
move.addCollisionObjects()

RGBD_r = ReconstructionSystem()
Tripo_r = LRM_Reconstruction()

pose_list = [7, 9, 21]
poses = move.getTFPose(pose_list)

for i in range(len(poses)):

    move.move_to_coords(poses[i])

    colorImage = rospy.wait_for_message(RGBD_r.color_topic, Image, timeout=20)
    depthImage = rospy.wait_for_message(RGBD_r.depth_topic, Image, timeout=20)
    camInfo = rospy.wait_for_message(RGBD_r.cam_topic, CameraInfo, timeout=20)
    
    colour_img = RGBD_r.color_callback(colorImage)
    depth_img = RGBD_r.depth_callback(depthImage)
    currPCD = RGBD_r.cam_info_callback(colour_img, depth_img, camInfo)

    colour_name = "/home/ivokosa/Desktop/3D_Mesh/colour_" + str(i) + ".png"
    depth_name = "/home/ivokosa/Desktop/3D_Mesh/depth_" + str(i) + ".png"

    cv2.imwrite(colour_name, colour_img)
    cv2.imwrite(depth_name, depth_img)

    # tripo_mesh = o3d.geometry.TriangleMesh()
    # tripo_mesh = Tripo_r.runner([colour_name])

    tfBuffer1 = tf2_ros.Buffer()
    listener1 = tf2_ros.TransformListener(tfBuffer1)
    Tw_0 = RGBD_r.pose2HTM( (tfBuffer1.lookup_transform('panda_link0', "tf_d0", rospy.Time(0), rospy.Duration(10.0))).transform)

    transformation = RGBD_r.getVSTF()
    regPCD = RGBD_r.draw_registration_result(currPCD, RGBD_r.regPCD, transformation, Tw_0)

origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.00001) 
regPCD = RGBD_r.outlierRemoval(regPCD,1,"total")
o3d.io.write_point_cloud("/home/ivokosa/Desktop/3D_Mesh/pc_Final.ply", regPCD)

if RGBD_r.visualisations:
    o3d.visualization.draw_geometries([regPCD,origin],point_show_normal=True,zoom=0.7,front=[ 0.0, 0.0, -1.0], lookat=[-8.947535058590317e-07, 3.6505334648302034e-05,0.00028049998945789412], up=[0.0, -1.0,0.0])
        
