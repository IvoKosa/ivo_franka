import os
import re
import numpy as np
import pandas as pd
import open3d as o3d
import probreg as cpd
import point_cloud_utils as pcu

class Evaluator:

    def __init__(self, obj1, obj2, pc1=None, pc2=None, scale=None, Orientation=None):

        # Inputs:
        # Objects 1 and 2 and optionally their corresponding point clouds
        # Scale: float to scale object 2 to object 1
        # Orientation: list of tuples (xyz euler angles) to rotate object 2 to object 1

        self.obj1 = obj1
        self.obj2 = obj2

        if pc1 == None:
            self.pc1 = self.obj2pcd(self.obj1)
        else:
            self.pc1 = pc1
        if pc2 == None:
            self.pc2 = self.obj2pcd(self.obj2)
        else:
            self.pc2 = pc2

        if scale == None:
            scale1 = self.get_scale(self.obj1)
            scale2 = self.get_scale(self.obj2)

            self.obj1.scale(scale1, center=self.obj1.get_center())
            self.obj2.scale(scale2, center=self.obj2.get_center())
            self.pc1.scale(scale1, center=self.pc1.get_center())
            self.pc2.scale(scale2, center=self.pc2.get_center())
        else:
            self.obj2.scale(scale, center=self.obj2.get_center())
            self.pc2.scale(scale, center=self.pc2.get_center())

        if Orientation == None:
            self.cpd_allignment()
        else:
            self.orient_match(Orientation)

        self.data = {
            "Chamfer Distance": [],
            "Hausdorff Distance": [],
            "Earth Movers Distance": [],
            "SA - Diff": []
        }

    def cpd_allignment(self):
        # tf_param, _, _ = cpd.cpd.registration_cpd(self.pc2, self.pc1)
        tf_param, _, _ = cpd.filterreg.registration_filterreg(self.pc2, self.pc1)
        self.pc2 = self.pc2.translate(tf_param.t)
        self.pc2 = self.pc2.rotate(tf_param.rot, center=self.pc2.get_center())

    def orient_match(self, rotations):

        centre = self.pc1.get_center()
        self.pc2.translate(centre, False)

        for i in range(len(rotations)):
            rot_mat = self.pc2.get_rotation_matrix_from_xyz(rotations[i])
            self.pc2.rotate(rot_mat, center=self.pc2.get_center())

    def get_scale(self, obj):
        bounding_box = obj.get_minimal_oriented_bounding_box()
        bound_array = np.asarray(bounding_box.get_max_bound() - bounding_box.get_min_bound())
        scalar1 =  1 / bound_array[0]
        scalar2 =  1 / bound_array[1]
        scalar3 =  1 / bound_array[2]

        avg_scalar = (scalar1 + scalar2 + scalar3) / 3
        return avg_scalar
    
    def obj2pcd(self, obj):
        # xyz = np.asarray(obj.vertices)
        # pc = o3d.geometry.PointCloud()
        # pc.points = o3d.utility.Vector3dVector(xyz)
        return obj.sample_points_uniformly(number_of_points=1000)
    
    def metrics(self):
        
        # self.pc1.paint_uniform_color([1, 0.706, 0])
        # self.pc2.paint_uniform_color([0, 1, 0])
        # o3d.visualization.draw_geometries([self.pc1, self.pc2])

        sa1 = self.obj1.get_surface_area()
        sa2 = self.obj2.get_surface_area()
        sa_diff = np.abs(sa1 - sa2)
        self.data["SA - Diff"].append(sa_diff)
        # self.data["Surface Area"].append(sa1)
        # self.data["Surface Area"].append(sa2)

        pcd_arr1 = np.asarray(self.pc1.points)
        pcd_arr2 = np.asarray(self.pc2.points)

        self.data["Chamfer Distance"].append(pcu.chamfer_distance(pcd_arr1, pcd_arr2))
        self.data["Hausdorff Distance"].append(pcu.hausdorff_distance(pcd_arr1, pcd_arr2))
        emd, _ = pcu.earth_movers_distance(pcd_arr1, pcd_arr2)
        self.data["Earth Movers Distance"].append(emd)

        return self.data

if __name__ == "__main__":

    def sorter(fileName):
        return int(str(fileName)[7:-4])

    # pwd = "/home/ivokosa/Desktop/Results/teapot_3_views/"
    # gt_obj = o3d.io.read_triangle_mesh("/home/ivokosa/model_editor_models/utah_teapot/teapot.obj")

    main_dir = "/home/ivokosa/Desktop/Results/"
    sub_dir = "bear_all_views"
    pwd = main_dir + sub_dir + "/"
    gt_obj = o3d.io.read_triangle_mesh("/home/ivokosa/model_editor_models/teddy_bear/TeddyBear-fixed.obj")

    pc_dir = os.path.join(pwd, "point_clouds/")
    mesh_dir = os.path.join(pwd, "meshes/")
    pc_files = os.listdir(pc_dir)
    pc_files.sort(key=sorter)

    rgbd_rot = [(np.pi * 1.5, 0, 0), (0, 0, np.pi / 2), (0, np.pi / 1.1, 0)]
    tripo_rot = [(np.pi / 1.8, np.pi / 1.5, 0), (0, np.pi * 2.1, 0)]

    data = {
        "Img / Reconstruction Method": [],
        "Chamfer Distance": [],
        "Hausdorff Distance": [],
        "Earth Movers Distance": [],
        "SA - Diff": []
    }
    
    for i, file in enumerate(pc_files):

        num = int(str(file)[7:-4])

        rgbd_pc = o3d.io.read_point_cloud(os.path.join(pc_dir ,file))

        rgdb_fileName = "rgbdMesh_" + str(num) + ".obj"
        rgbd_obj = o3d.io.read_triangle_mesh(os.path.join(mesh_dir , rgdb_fileName))
        
        tripo_fileName = "tripoMesh_" + str(num) + ".obj"
        tripo_obj = o3d.io.read_triangle_mesh(os.path.join(mesh_dir , tripo_fileName))

        # ------------- ------------- Evaluating GT - RGBD ------------- -------------

        eval = Evaluator(gt_obj, rgbd_obj)
        metrics = eval.metrics()

        for key, value in (metrics).items():
            (data)[key].extend(value)

        RGBD_Img = "Img_" + str(num) + "_RGBD-GT"
        data["Img / Reconstruction Method"].append(RGBD_Img)

        # ------------- ------------- Evaluating GT - TripoSR ------------- -------------

        eval2 = Evaluator(gt_obj, tripo_obj)
        metrics = eval2.metrics()

        for key, value in (metrics).items():
            (data)[key].extend(value)

        LRM_Img = "Img_" + str(num) + "_LRM-GT"
        data["Img / Reconstruction Method"].append(LRM_Img)

        # print(data)

    df = pd.DataFrame(data)
    df.to_csv("/home/ivokosa/" + sub_dir + ".csv")
