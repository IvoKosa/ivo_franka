#!/usr/bin/env python3

import os
import numpy as np
import open3d as o3d
import pyvista as pv
import point_cloud_utils as pcu

output_dir = "/home/ivokosa/Desktop/Reconst_Output/"
run_number = 4

full_dir = os.path.join(output_dir, str(run_number))
new_dir = full_dir + "/metrics/"

# os.makedirs(new_dir)
# f = open("metrics.txt", "x")
# f.close()

gt_obj = o3d.io.read_triangle_mesh("/home/ivokosa/model_editor_models/utah_teapot/teapot.obj")
gt_pc = gt_obj.sample_points_uniformly(number_of_points=500)

pc_dir = os.path.join(full_dir, "point_clouds/")
mesh_dir = os.path.join(full_dir, "meshes/")

pc_files = os.listdir(pc_dir)
pc_files.sort()

# ------------- ------------- Main Loop ------------- -------------

for i, file in enumerate(pc_files):

    comparison_metrics = {}

    # Files for comparison:
    #   gt_obj
    #   gt_pc
    #   rgbd_mesh
    #   tripo_mesh
    #   rgbd_pc
    #   tripo_pc

    # ------------- ------------- Loading Files ------------- -------------

    rgbd_pc = o3d.io.read_point_cloud(os.path.join(pc_dir ,file))

    if i == (len(pc_files) - 1):

        comparison_metrics["Image Number"] = "Final"

        rgbd_mesh = o3d.io.read_triangle_mesh(os.path.join(mesh_dir, "rgbdMesh_final.obj"))
        tripo_fileName = "tripoMesh_" + str((i - 1)) + ".obj"
        tripo_mesh = o3d.io.read_triangle_mesh(os.path.join(mesh_dir , tripo_fileName))

    else:

        comparison_metrics["Image Number"] = i

        rgdb_fileName = "rgbdMesh_" + str(i) + ".obj"
        rgbd_mesh = o3d.io.read_triangle_mesh(os.path.join(mesh_dir , rgdb_fileName))
        tripo_fileName = "tripoMesh_" + str(i) + ".obj"
        tripo_mesh = o3d.io.read_triangle_mesh(os.path.join(mesh_dir , tripo_fileName))

    tripo_pc = tripo_mesh.sample_points_uniformly(number_of_points=500)

    # ------------- ------------- Scaling Objects Uniformly ------------- -------------

    gb = gt_obj.get_minimal_oriented_bounding_box()
    rb = rgbd_mesh.get_minimal_oriented_bounding_box()
    tb = tripo_mesh.get_minimal_oriented_bounding_box()

    gn = np.asarray(gb.get_max_bound() - gb.get_min_bound())
    rn = np.asarray(rb.get_max_bound() - rb.get_min_bound())
    tn = np.asarray(tb.get_max_bound() - tb.get_min_bound())

    comparison_metrics["GT Scaler"] = 1 / gn[0]
    comparison_metrics["RGBD Scaler"] = 1 / rn[0]
    comparison_metrics["TripoSR Scaler"] = 1 / tn[0]

    gt_pc.scale(comparison_metrics["GT Scaler"], center=gt_pc.get_center())
    gt_obj.scale(comparison_metrics["GT Scaler"], center=gt_obj.get_center())

    rgbd_pc.scale(comparison_metrics["RGBD Scaler"], center=rgbd_pc.get_center())
    rgbd_mesh.scale(comparison_metrics["RGBD Scaler"], center=rgbd_mesh.get_center())

    tripo_pc.scale(comparison_metrics["TripoSR Scaler"], center=tripo_pc.get_center())
    tripo_mesh.scale(comparison_metrics["TripoSR Scaler"], center=tripo_mesh.get_center())

    # ------------- ------------- Model Comparison ------------- -------------

    
    