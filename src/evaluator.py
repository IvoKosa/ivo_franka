import os
import numpy as np
import pandas as pd
import open3d as o3d
import probreg as cpd
import point_cloud_utils as pcu

class Evaluator:

    def __init__(self, obj1, obj2, pc1=None, pc2=None, scale=None, Orientation=None):
        self.obj1 = self.rescale(obj1)
        self.obj2 = self.rescale(obj2)
        
        self.pc1 = o3d.geometry.PointCloud()
        self.pc1 = o3d.geometry.PointCloud()

        if pc1 == None:
            self.pc1 = self.obj2pcd(self.obj1)
        else:
            if scale == None:
                self.pc1 = (self.rescale(pc1))
            else:
                self.pc1 = self.pc1.scale(scale[0], center=self.pc1.get_center())

        if pc2 == None:
            self.pc2 = self.obj2pcd(self.obj2)
        else:
            if scale == None:
                self.pc2 = (self.rescale(pc2))
            else:
                self.pc2 = self.pc2.scale(scale[1], center=self.pc2.get_center())

        if Orientation == None:
            self.cpd_allignment()
        else:
            self.orient_match(Orientation)

        self.data = {
            "SA - Diff": [],
            "Chamfer Distance": [],
            "Hausdorff Distance": [],
            "Earth Movers Distance": []
        }

    def cpd_allignment(self):
        tf_param, _, _ = cpd.cpd.registration_cpd(self.pc2, self.pc1)
        self.pc2.points = tf_param.transform(self.pc2.points)

    def orient_match(self, rotations):

        centre = self.pc1.get_center()
        self.pc2.translate(centre, False)

        for i in range(len(rotations)):
            rot_mat = self.pc2.get_rotation_matrix_from_xyz(rotations[i])
            self.pc2.rotate(rot_mat, center=self.pc2.get_center())

    def rescale(self, obj):
        bounding_box = obj.get_minimal_oriented_bounding_box()
        bound_array = np.asarray(bounding_box.get_max_bound() - bounding_box.get_min_bound())
        scalar1 =  1 / bound_array[0]
        scalar2 =  1 / bound_array[1]
        scalar3 =  1 / bound_array[2]

        avg_scalar = (scalar1 + scalar2 + scalar3) / 3

        obj.scale(avg_scalar, center=obj.get_center())
        return obj
    
    def obj2pcd(self, obj):
        xyz = np.asarray(obj.vertices)
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(xyz)
        return pc
    
    def metrics(self):

        o3d.visualization.draw_geometries([self.pc1, self.pc2])

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

if __name__ == "__main__":

    pwd = "/home/ivokosa/Desktop/Results/Teapot_3_Views/"
    gt_obj = o3d.io.read_triangle_mesh("/home/ivokosa/model_editor_models/utah_teapot/teapot.obj")

    pc_dir = os.path.join(pwd, "point_clouds/")
    mesh_dir = os.path.join(pwd, "meshes/")
    pc_files = os.listdir(pc_dir)
    pc_files.sort()

    rgbd_rot = [(np.pi * 1.5, 0, 0), (0, 0, np.pi / 2), (0, np.pi / 1.1, 0)]
    tripo_rot = [(np.pi / 1.8, np.pi / 1.5, 0), (0, np.pi * 2.1, 0)]

    for i, file in enumerate(pc_files):

        rgbd_pc = o3d.io.read_point_cloud(os.path.join(pc_dir ,file))

        if i == (len(pc_files) - 1):

            rgbd_obj = o3d.io.read_triangle_mesh(os.path.join(mesh_dir, "rgbdMesh_final.obj"))
            tripo_fileName = "tripoMesh_" + str((i - 1)) + ".obj"
            tripo_obj = o3d.io.read_triangle_mesh(os.path.join(mesh_dir , tripo_fileName))

        else:

            rgdb_fileName = "rgbdMesh_" + str(i) + ".obj"
            rgbd_obj = o3d.io.read_triangle_mesh(os.path.join(mesh_dir , rgdb_fileName))
            tripo_fileName = "tripoMesh_" + str(i) + ".obj"
            tripo_obj = o3d.io.read_triangle_mesh(os.path.join(mesh_dir , tripo_fileName))

        # ------------- ------------- Evaluating GT - RGBD ------------- -------------

        eval = Evaluator(gt_obj, rgbd_obj, rgbd_rot, None, rgbd_pc)
        eval.metrics()

        # ------------- ------------- Evaluating GT - TripoSR ------------- -------------

        tripo_pc = tripo_obj.sample_points_uniformly(number_of_points=500)

        eval2 = Evaluator(gt_obj, tripo_obj, tripo_rot, None, tripo_pc, )
        eval2.metrics()

        for key, value in (eval2.data).items():
            (eval.data)[key].extend(value)

        # ------------- ------------- Evaluating GT - TripoSR ------------- -------------

        # eval3 = Evaluator(rgbd_obj, tripo_obj, rgbd_pc, tripo_pc)
        # eval3.metrics()

        # for key, value in (eval3.data).items():
        #     (eval.data)[key].extend(value)

        # ------------- ------------- Saving ------------- ------------- -------------
        print(eval.data)

        df = pd.DataFrame(eval.data, index = ["GT - RGBD", "GT - TripoSR"]) # "RGBD - TripoSR" index = ["GT - RGBD", "GT - TripoSR"]

        print(df)
        img_name = "Image_" + str(i) + ".csv"
        df.to_csv(img_name)
