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

# Own Imports
from reconstruction import ReconstructionSystem

class Evaluation:

    def __init__(self) -> None:
        pass

    def hausdorff(self, pc1, pc2):
        return pcu.hausdorff_distance(pc1, pc2)
    
    def histogram_similarity(self, model1, model2):

        bins = [20, 20, 20]
        histogram1, _ = np.histogramdd(model1, bins=bins)
        histogram2, _ = np.histogramdd(model2, bins=bins)

        histogram1 /= np.sum(histogram1)
        histogram2 /= np.sum(histogram2)

        return wasserstein_distance(histogram1.flatten(), histogram2.flatten())

    def visual_comparison(self, model1, model2):

        p = pv.Plotter(shape=(1, 2))
        p.add_mesh(model1)
        p.subplot(0, 1)
        p.add_mesh(model2)
        p.link_views()
        p.view_isometric()
        p.show()

if __name__ == "__main__":

    rospy.init_node("Evaluation", anonymous=True)

    view_nums1 = [16, 7, 8, 9, 13] # [7, 9, 21]  9 
    view_nums2 = [7]

    reconstruct = ReconstructionSystem()
    evaluate = Evaluation()

    r1 = reconstruct.runner(view_nums1)

    o3d.io.write_point_cloud("cyl.ply", r1)

    # r2 = reconstruct.runner(view_nums2)

    # evaluate.visual_comparison(m1, m2)
