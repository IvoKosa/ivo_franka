import os
import numpy as np
import open3d as o3d
import pandas as pd
import point_cloud_utils as pcu

class Evaluator:

    def __init__(self, obj1, obj2, pc1=None, pc2=None):
        self.obj1 = self.rescale(obj1)
        self.obj2 = self.rescale(obj2)

        if pc1 == None:
            self.pc1 = np.asarray(self.obj2pcd(self.obj1).points)
        else:
            self.pc1 = np.asarray(self.rescale(pc1).points)
        if pc2 == None:
            self.pc2 = np.asarray(self.obj2pcd(self.obj2).points)
        else:
            self.pc2 = np.asarray(self.rescale(pc2).points)

        self.data = {
            "Surface Area": [],
            "SA - Diff": [],
            "Chamfer Distance": [],
            "Hausdorff Distance": [],
            "Earth Movers Distance": []
        }

    def rescale(self, obj):
        bounding_box = obj.get_minimal_oriented_bounding_box()
        bound_array = np.asarray(bounding_box.get_max_bound() - bounding_box.get_min_bound())
        scalar =  1 / bound_array[0]
        obj.scale(scalar, center=obj.get_center())
        return obj
    
    def obj2pcd(self, obj):
        xyz = np.asarray(obj.vertices)
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(xyz)
        return pc
    
    def metrics(self):
        sa1 = self.obj1.get_surface_area()
        sa2 = self.obj2.get_surface_area()
        sa_diff = np.abs(sa1 - sa2)

        self.data["Surface Area"].append(sa1)
        self.data["Surface Area"].append(sa2)
        self.data["SA - Diff"].append(sa_diff)
        self.data["SA - Diff"].append(None)
        self.data["Chamfer Distance"].append(pcu.chamfer_distance(self.pc1, self.pc2))
        self.data["Chamfer Distance"].append(None)
        self.data["Hausdorff Distance"].append(pcu.hausdorff_distance(self.pc1, self.pc2))
        self.data["Hausdorff Distance"].append(None)
        emd, _ = pcu.earth_movers_distance(self.pc1, self.pc2)
        self.data["Earth Movers Distance"].append(emd)
        self.data["Earth Movers Distance"].append(None)

if __name__ == "__main__":

    pwd = "/home/ivokosa/Desktop/Results/Teapot_3_Views/"
    gt_obj = o3d.io.read_triangle_mesh("/home/ivokosa/model_editor_models/utah_teapot/teapot.obj")

    pc_dir = os.path.join(pwd, "point_clouds/")
    mesh_dir = os.path.join(pwd, "meshes/")
    pc_files = os.listdir(pc_dir)
    pc_files.sort()

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

        eval = Evaluator(gt_obj, rgbd_obj, None, rgbd_pc)
        eval.metrics()

        # ------------- ------------- Evaluating GT - TripoSR ------------- -------------

        tripo_pc = tripo_obj.sample_points_uniformly(number_of_points=500)

        eval2 = Evaluator(gt_obj, tripo_obj, None, tripo_pc)
        eval2.metrics()

        for key, value in (eval2.data).items():
            (eval.data)[key].extend(value)

        # ------------- ------------- Evaluating GT - TripoSR ------------- -------------

        eval3 = Evaluator(rgbd_obj, tripo_obj, rgbd_pc, tripo_pc)
        eval3.metrics()

        for key, value in (eval3.data).items():
            (eval.data)[key].extend(value)

        # ------------- ------------- Saving ------------- ------------- -------------

        df = pd.DataFrame(eval.data, index = ["Ground Truth", "RGBD", "Ground Truth", "TripoSR", "RGBD", "TripoSR"])

        print(df)
        img_name = "Image_" + str(i) + ".csv"
        df.to_csv(img_name)
