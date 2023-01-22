# 3D Object Detection Pipeline
An unsupervised 3D Object detection pipeline using 3D LiDAR points 

## Overview
<p align="center">
<img src="https://github.com/varshasathya/Lidar-3D-Object-Detection/blob/main/output/overview.PNG" width=640/>
image source: thinkautonomous.ai
</p>

### Input Point Cloud
<p align="center">
<img src="https://github.com/varshasathya/Lidar-3D-Object-Detection/blob/main/output/pcd.PNG" width=640>
</p>

### Downsampling the point clouds
<p align="center">
<img src="https://github.com/varshasathya/Lidar-3D-Object-Detection/blob/main/output/downsampled.PNG" width=640/>
</p>

### Segment the point clouds as inliers (objects of interest) and outliers (ground) using RANSAC algorithm
<p align="center">
<img src="https://github.com/varshasathya/Lidar-3D-Object-Detection/blob/main/output/ransac.PNG" width=640/>
</p>

### Clustering using DBSCAN
<p align="center">
<img src="https://github.com/varshasathya/Lidar-3D-Object-Detection/blob/main/output/dbscan.PNG" width=640/>
</p>

### Compute 3D bounding box for each cluster
<p align="center">
<img src="https://github.com/varshasathya/Lidar-3D-Object-Detection/blob/main/output/bbox.PNG" width=640/>
</p>

## Video Result on 9 pcd samples
<p align="center">
<img src="https://github.com/varshasathya/Lidar-3D-Object-Detection/blob/main/output/movie.gif" width=640/>
</p>

