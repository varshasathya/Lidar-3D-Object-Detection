import numpy as np
import time
import open3d as o3d
import pandas as pd
import matplotlib.pyplot as plt

def downsample(pcd,voxel_size=0.2):
    pcd_downsampled = pcd.voxel_down_sample(voxel_size = voxel_size)
    return pcd_downsampled  

def ransac(pcd,distance_threshold = 0.3, num_iterations=100):
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold, ransac_n=3, num_iterations=num_iterations)
    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([0, 1.0, 1.0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    outlier_cloud.paint_uniform_color([1.0, 0, 0])
    return inlier_cloud,outlier_cloud

def dbscan(outlier_cloud, eps=0.45, min_points=7):

    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(outlier_cloud.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")

    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    return outlier_cloud, labels

def oriented_bbox(outlier_cloud, labels, min_points=30, max_points=400):
    obbs = []
    indexes = pd.Series(range(len(labels))).groupby(labels,sort = False).apply(list).tolist()

    for i in range(len(indexes)):
        nb_points = len(outlier_cloud.select_by_index(indexes[i]).points)
        if (nb_points > min_points and nb_points < max_points):
            sub_cloud = outlier_cloud.select_by_index(indexes[i])
            obb = sub_cloud.get_axis_aligned_bounding_box()
            obb.color = (0,0,1)
            obbs.append(obb)
    print("number of bounding boxes detected:",len(obbs))
    return obbs

def object_detection_pipeline(pcd, voxel_size=0.25, distance_threshold=0.3, num_iterations=100, eps=0.4, min_points=5):
    pcd = o3d.io.read_point_cloud(pcd)
    pcd_downsampled = downsample(pcd, voxel_size)
    inlier_cloud,outlier_cloud = ransac(pcd_downsampled, distance_threshold=distance_threshold, num_iterations=num_iterations)
    outlier_cloud, labels = dbscan(outlier_cloud, eps=0.45, min_points=7)
    obbs = oriented_bbox(outlier_cloud, labels)

    list_of_visuals = []
    list_of_visuals.append(outlier_cloud)
    list_of_visuals.extend(obbs)
    list_of_visuals.append(inlier_cloud)
    return list_of_visuals

if __name__ == "__main__":
    pcd_file = "test_files/UDACITY/0000000000.pcd"
    list_of_visuals = object_detection_pipeline(pcd_file)
    o3d.visualization.draw_geometries(list_of_visuals)