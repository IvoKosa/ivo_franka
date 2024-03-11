import open3d as o3d
import numpy as np

dataset = o3d.data.LivingRoomPointClouds()
pcds = []
for pcd_path in dataset.paths:
    pcds.append(o3d.io.read_point_cloud(pcd_path))
