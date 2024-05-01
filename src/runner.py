#!/usr/bin/env python3

# General Imports
import cv2
import os
import numpy as np
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

move = MoveGroupPyInterface()
move.addCollisionObjects()

teapot_object_size = [0.14,0.14,0.1]
teapot_object_position = [0.650221, 0]

# teapot_object_size = [0.08,0.08,0.08]
# teapot_object_position = [0.649997, 0]

RGBD_r = ReconstructionSystem(teapot_object_size, teapot_object_position)
# Tripo_r = LRM_Reconstruction()

pose_list = [7, 9, 21]
poses = move.getTFPose()

for i in range(len(poses)):

    success = move.move_to_coords(poses[i])

    if success[1] == False:
        continue

    # ------------- ------------- Getting Image Info ------------- -------------

    colorImage = rospy.wait_for_message(RGBD_r.color_topic, Image, timeout=20)
    depthImage = rospy.wait_for_message(RGBD_r.depth_topic, Image, timeout=20)
    camInfo = rospy.wait_for_message(RGBD_r.cam_topic, CameraInfo, timeout=20)
    
    colour_img = RGBD_r.color_callback(colorImage)
    depth_img = RGBD_r.depth_callback(depthImage)
    currPCD = RGBD_r.cam_info_callback(colour_img, depth_img, camInfo)

    colour_name = new_dir + "/images/" + "colour_" + str(i) + ".png"
    depth_name = new_dir + "/images/" + "depth_" + str(i) + ".png"

    cv2.imwrite(colour_name, colour_img)
    cv2.imwrite(depth_name, depth_img)

    # If using Tripo:
    # tripo_mesh_name = new_dir + "/meshes/tripoMesh_" + str(i) + ".obj"
    # tripo_mesh = o3d.geometry.TriangleMesh()
    # tripo_mesh = Tripo_r.runner([colour_name])
    # o3d.io.write_triangle_mesh(tripo_mesh_name, tripo_mesh)

    # ------------- ------------- Generating Point Cloud ------------- -------------

    tfBuffer1 = tf2_ros.Buffer()
    listener1 = tf2_ros.TransformListener(tfBuffer1)
    Tw_0 = RGBD_r.pose2HTM( (tfBuffer1.lookup_transform('panda_link0', "tf_d0", rospy.Time(0), rospy.Duration(10.0))).transform)

    transformation = RGBD_r.getVSTF()
    RGBD_r.regPCD = RGBD_r.draw_registration_result(currPCD, RGBD_r.regPCD, transformation, Tw_0)

    # ------------- ------------- Saving Point Cloud ------------- -------------

    pc_name = new_dir + "/point_clouds/rgdbPc_" + str(i) + ".ply"
    o3d.io.write_point_cloud(pc_name, RGBD_r.regPCD)

    # ------------- ------------- Generating Mesh from Point Cloud ------------- -------------

    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.00001) 
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(RGBD_r.regPCD, depth=8)

    vertices_to_remove = densities < np.quantile(densities, 0.01)
    mesh.remove_vertices_by_mask(vertices_to_remove)

    if RGBD_r.visualisations:
        o3d.visualization.draw_geometries([mesh,origin],zoom=0.7,front=[ 0.0, 0.0, -1.0], lookat=[-8.947535058590317e-07, 3.6505334648302034e-05,0.00028049998945789412], up=[0.0, -1.0,0.0])
    mesh.compute_triangle_normals()

    # ------------- ------------- Saving Mesh ------------- -------------

    mesh_name = new_dir + "/meshes/rgbdMesh_" + str(i) + ".obj"
    o3d.io.write_triangle_mesh(mesh_name, mesh)

# ------------- ------------- End Loop ------------- -------------

origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.00001) 
RGBD_r.regPCD = RGBD_r.outlierRemoval(RGBD_r.regPCD,1,"total")

pc_name = new_dir + "/point_clouds/rgdbPc_final" + ".ply"
o3d.io.write_point_cloud(pc_name, RGBD_r.regPCD)

mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(RGBD_r.regPCD, depth=8)
vertices_to_remove = densities < np.quantile(densities, 0.01)
mesh.remove_vertices_by_mask(vertices_to_remove)
mesh_name = new_dir + "/meshes/rgbdMesh_final" + ".obj"
o3d.io.write_triangle_mesh(mesh_name, mesh)

if RGBD_r.visualisations:
    o3d.visualization.draw_geometries([RGBD_r.regPCD,origin],point_show_normal=True,zoom=0.7,front=[ 0.0, 0.0, -1.0], lookat=[-8.947535058590317e-07, 3.6505334648302034e-05,0.00028049998945789412], up=[0.0, -1.0,0.0])
