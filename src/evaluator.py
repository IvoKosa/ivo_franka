import os
import pprint
import numpy as np
import open3d as o3d
import pyvista as pv
import point_cloud_utils as pcu

class Evaluator:

    def __init__(self, obj1, obj2):
        self.obj1 = obj1
        self.obj2 = obj2

    def rescale(self, obj):
        bounding_box = obj.get_minimal_oriented_bounding_box()
        bound_array = np.asarray(bounding_box.get_max_bound() - bounding_box.get_min_bound())
        scalar =  1 / bound_array[0]
        obj.scale(scalar, center=obj.get_center())
        return obj

if __name__ == "__main__":

    pwd = "/home/ivokosa/Desktop/Reconst_Output/6/meshes/rgbdMesh_0.obj"

    gt_obj = o3d.io.read_triangle_mesh

    eval = Evaluator()
