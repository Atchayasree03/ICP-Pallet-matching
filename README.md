# G+1 Pallet Detection

A point-cloud-based pallet detection and template matching system using **PCL** and **Open3D C++**.

The system is designed for a depth camera mounted above the forklift forks. The camera captures a depth image, which is converted into a point cloud. The pallet region is then processed using ROI filtering, DBSCAN clustering, fork-entry extraction, template matching, ICP alignment, and finally pallet pose estimation.

---

## 1. Project Objective

The objective of this project is to detect and identify a pallet from a 3D point cloud without using YOLO or any image-based object detector.

The system uses:

- Depth camera point cloud
- Manually defined XYZ ROI
- DBSCAN clustering
- Fork-entry extraction
- 3D pallet templates
- PCA-based initial alignment
- ICP registration
- Fitness score
- RMSE
- Pallet center
- Pallet rotation angle

The final goal is to automatically determine:

1. Which pallet is present
2. Which template matches the pallet
3. Where the pallet center is
4. What its orientation/angle is

---

# 2. Overall Pipeline

```text
                    Depth Camera
                         |
                         v
                    Raw Point Cloud
                         |
                         v
                 XYZ ROI Filtering
                         |
                         v
                ROI Point Cloud (PCD)
                         |
                         v
                  Convert m -> mm
                         |
                         v
                    DBSCAN
                         |
                         v
                 Pallet Cluster
                         |
                         v
                Fork Entry Extraction
                         |
                         v
                 Scene Point Cloud
                         |
                         v
              Template Front Plane
                         |
                         v
                 PCA Initialization
                         |
                         v
                       ICP
                         |
             +-----------+-----------+
             |           |           |
             v           v           v
          Fitness       RMSE     Transformation
                                     |
                                     v
                              Center + Angle
