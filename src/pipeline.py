#!/usr/bin/env python3

# General Imports
import cv2
import os
import numpy as np
import pandas as pd
import open3d as o3d

# ROS Imports 
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

# Own Imports
from franka_move import MoveGroupPyInterface
from rgbd_reconstruct import RGBD_Reconstruction
from LRM_reconstruct import LRM_Reconstruction
from evaluator import Evaluator

# ------------- ------------- Main ------------- -------------

# For further use: 
# Change directory output_dir to desired location
# This will create sequential numerated folders for each time the pipeline is run, with the following folder structure:
# output_dir:
#     |_ run_number
#           |_ images (images labled as: "colour_(viewpoint_num).png" or "depth_(viewpoint_num).png")
#           |_ meshes (meshes labled as: "LRM_Mesh_(viewpoint_num).obj" or "rgbdMesh_(viewpoint_num).obj")
#           |_ point_clouds (point clouds labled as: "rgbdPc_(viewpoint_num).ply")
#
# Use of the pipeline as is will automatically implement both reconstruction methods for the viewpoints selected 
# and compare them to the ground truth

rospy.init_node("pipeline_runner", anonymous=True)

output_dir = "/home/ivokosa/Desktop/Reconstruction_output/"

os.makedirs(output_dir, exist_ok=True)

folders = os.listdir(output_dir)
max_num = 0

for i in folders:
    try:
        folder_num = int(i)
    except ValueError:
        continue
    
    if folder_num > max_num:
        max_num = folder_num

new_dir = os.path.join(output_dir, str(max_num + 1))
print(new_dir)
os.makedirs(new_dir)
os.makedirs(os.path.join(new_dir, "images"))
os.makedirs(os.path.join(new_dir, "point_clouds"))
os.makedirs(os.path.join(new_dir, "meshes"))

bridge = CvBridge()

move = MoveGroupPyInterface()
move.addCollisionObjects()

teapot_object_size = [0.14,0.14,0.14]
teapot_object_position = [0.650221, 0]

RGBD_r = RGBD_Reconstruction(teapot_object_size, teapot_object_position)

LRM_r = LRM_Reconstruction() # Comment out if not using LRM:

pose_list = [7, 11, 21]
poses = move.getTFPose(pose_list)

data_rgbd = {
        "Img / Reconstruction Method": [],
        "Chamfer Distance": [],
        "Hausdorff Distance": [],
        "Earth Movers Distance": [],
        "SA - Diff": []
    }

data_lrm = {
    "Img / Reconstruction Method": [],
    "Chamfer Distance": [],
    "Hausdorff Distance": [],
    "Earth Movers Distance": [],
    "SA - Diff": []
}

gt_obj = o3d.io.read_triangle_mesh("/home/ivokosa/ros/noetic/system/src/ivo_franka/src/model_editor_models/utah_teapot/teapot.obj")

for i in range(len(poses)):

    success = move.move_to_coords(poses[i])

    if success[1] == False:
        continue

    # ------------- ------------- Getting Image Info ------------- -------------

    colorImage = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=20)
    depthImage = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image, timeout=20)
    camInfo = rospy.wait_for_message("/camera/aligned_depth_to_color/camera_info", CameraInfo, timeout=20)

    colour_img_rgb = bridge.imgmsg_to_cv2(colorImage, "rgb8")
    colour_img_bgr = bridge.imgmsg_to_cv2(colorImage, "bgr8")
    depth_img = bridge.imgmsg_to_cv2(depthImage, "32FC1")

    colour_name = new_dir + "/images/" + "colour_" + str(pose_list[i]) + ".png"
    depth_name = new_dir + "/images/" + "depth_" + str(pose_list[i]) + ".png"

    cv2.imwrite(colour_name, colour_img_bgr)
    cv2.imwrite(depth_name, depth_img)

    # ------------- ------------- Running Reconstructions and Saving Outputs ------------- -------------

    RGBD_r.runner(colour_img_rgb, depth_img, camInfo)

    RGBD_pc_name = new_dir + "/point_clouds/rgbdPc_" + str(pose_list[i]) + ".ply"
    o3d.io.write_point_cloud(RGBD_pc_name, RGBD_r.regPCD)

    RGBD_mesh_name = new_dir + "/meshes/rgbdMesh_" + str(pose_list[i]) + ".obj"
    o3d.io.write_triangle_mesh(RGBD_mesh_name, RGBD_r.regMesh)

    # Comment out from here onwards if not using LRM:

    LRM_mesh = LRM_r.runner(colour_name)
    LRM_mesh_name = new_dir + "/meshes/LRM_Mesh_" + str(pose_list[i]) + ".obj"
    o3d.io.write_triangle_mesh(LRM_mesh_name, LRM_mesh)
    
    # ------------- ------------- Gathering Metrics  ------------- -------------

    eval_GT_RGBD = Evaluator(gt_obj, RGBD_r.regMesh, None, RGBD_r.regPCD)
    eval_GT_LRM = Evaluator(gt_obj, LRM_mesh)

    RGBD_metrics = eval_GT_RGBD.metrics()
    LRM_metrics = eval_GT_LRM.metrics()

    # ------------- ------------- Appending RGBD ------------- -------------
    
    for key, value in (RGBD_metrics).items():
        (data_rgbd)[key].extend(value)

    RGBD_Img = "Img_" + str(i) + "_RGBD - GT"
    data_rgbd["Img / Reconstruction Method"].append(RGBD_Img)

    # ------------- ------------- Appending LRM ------------- -------------

    for key, value in (LRM_metrics).items():
        (data_lrm)[key].extend(value)

    LRM_Img = "Img_" + str(i) + "_LRM - GT"
    data_lrm["Img / Reconstruction Method"].append(LRM_Img)

df1 = pd.DataFrame(data_rgbd)
df1.to_csv("/home/ivokosa/rgbd_Metrics.csv")

df2 = pd.DataFrame(data_lrm)
df2.to_csv("/home/ivokosa/lrm_Metrics.csv")

