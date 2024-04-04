import pyvista as pv
import point_cloud_utils as pcu
import numpy as np
import trimesh as tr
from scipy.stats import wasserstein_distance

class Evaluation:

    def __init__(self) -> None:
        pass

    def hausdorff(pc1, pc2):
        return pcu.hausdorff_distance(pc1, pc2)
    
    def histogram_similarity(model1, model2):

        bins = [20, 20, 20]
        histogram1, _ = np.histogramdd(model1, bins=bins)
        histogram2, _ = np.histogramdd(model2, bins=bins)

        histogram1 /= np.sum(histogram1)
        histogram2 /= np.sum(histogram2)

        return wasserstein_distance(histogram1.flatten(), histogram2.flatten())

    def visual_comparison(model1, model2):

        p = pv.Plotter(shape=(1, 2))
        p.add_mesh(model1)
        p.subplot(0, 1)
        p.add_mesh(model2)
        p.link_views()
        p.view_isometric()
        p.show()

    def viewpoints():

        success = [7, 9, 21]
        single_angle = [0]
        