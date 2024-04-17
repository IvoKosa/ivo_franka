#!/usr/bin/env python3

# Ros Imports
import rospy

# Evaluation Metrics
import open3d as o3d
import pyvista as pv
import point_cloud_utils as pcu
import numpy as np
import trimesh as tr
from scipy.stats import wasserstein_distance

class Evaluation:

    def __init__(self) -> None:
        pass

    def hausdorff(self, pc1, pc2):
        return pcu.hausdorff_distance(pc1, pc2)
    
    def earth_movers(self, pc1, pc2):
        return pcu.earth_movers_distance(pc1, pc2)
    
    def chamfer(self, pc1, pc2):
        return pcu.chamfer_distance(pc1, pc2)
    
    def histogram_similarity(self, model1, model2):

        bins = [20, 20, 20]
        histogram1, _ = np.histogramdd(model1, bins=bins)
        histogram2, _ = np.histogramdd(model2, bins=bins)

        histogram1 /= np.sum(histogram1)
        histogram2 /= np.sum(histogram2)

        return wasserstein_distance(histogram1.flatten(), histogram2.flatten())
    
    def min_boundingBox(self):
        pass

    def visual_comparison(self, model1, model2):

        p = pv.Plotter(shape=(1, 2))
        p.add_mesh(model1)
        p.subplot(0, 1)
        p.add_mesh(model2)
        p.link_views()
        p.view_isometric()
        p.show()

if __name__ == "__main__":

    # rospy.init_node("Evaluation", anonymous=True)

    e = Evaluation()

    # pc = o3d.geometry.PointCloud()
    pc = o3d.io.read_point_cloud("teapot_pc.ply")
    msh = o3d.io.read_triangle_mesh("teapot_mesh.obj")

    # mesh = e.pc2mesh(pc)

    # e.visual_comparison(msh, msh)
