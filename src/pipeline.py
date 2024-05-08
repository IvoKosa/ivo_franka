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

rospy.init_node("pipeline_runner", anonymous=True)

output_dir = "/home/ivokosa/Desktop/Reconst_Output/"

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

# Uncomment if using Tripo:
Tripo_r = LRM_Reconstruction()

pose_list = [7, 9, 21]
poses = move.getTFPose(pose_list)

data = {
    "Img / Reconstruction Method": [],
    "Chamfer Distance": [],
    "Hausdorff Distance": [],
    "Earth Movers Distance": [],
    "SA - Diff": []
    }

gt_obj = o3d.io.read_triangle_mesh("/home/ivokosa/model_editor_models/utah_teapot/teapot.obj")

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

    colour_name = new_dir + "/images/" + "colour_" + str(i) + ".png"
    depth_name = new_dir + "/images/" + "depth_" + str(i) + ".png"

    cv2.imwrite(colour_name, colour_img_bgr)
    cv2.imwrite(depth_name, depth_img)

    # ------------- ------------- Running Reconstructions ------------- -------------

    RGBD_r.runner(colour_img_rgb, depth_img, camInfo)

    RGBD_pc_name = new_dir + "/point_clouds/rgdbPc_" + str(i) + ".ply"
    o3d.io.write_point_cloud(RGBD_pc_name, RGBD_r.regPCD)

    RGBD_mesh_name = new_dir + "/meshes/rgbdMesh_" + str(i) + ".obj"
    o3d.io.write_triangle_mesh(RGBD_mesh_name, RGBD_r.regMesh)

    LRM_mesh = Tripo_r.runner(colour_name)
    LRM_mesh_name = new_dir + "/meshes/tripoMesh_" + str(i) + ".obj"
    o3d.io.write_triangle_mesh(LRM_mesh_name, LRM_mesh)
    
    # ------------- ------------- Gathering Metrics  ------------- -------------

    eval_GT_RGBD = Evaluator(gt_obj, RGBD_r.regMesh, None, RGBD_r.regPCD)
    eval_GT_LRM = Evaluator(gt_obj, LRM_mesh)

    RGBD_metrics = eval_GT_RGBD.metrics()
    LRM_metrics = eval_GT_LRM.metrics()

    # ------------- ------------- Appending RGBD ------------- -------------
    
    for key, value in (RGBD_metrics).items():
        (data)[key].extend(value)

    RGBD_Img = "Img_" + str(i) + "_RGBD"
    data["Img / Reconstruction Method"].append(RGBD_Img)

    # ------------- ------------- Appending LRM ------------- -------------

    for key, value in (LRM_metrics).items():
        (data)[key].extend(value)

    LRM_Img = "Img_" + str(i) + "_LRM"
    data["Img / Reconstruction Method"].append(LRM_Img)

    print(data)
    df = pd.DataFrame(data)
    df.to_csv("/home/ivokosa/fullRun.csv")
