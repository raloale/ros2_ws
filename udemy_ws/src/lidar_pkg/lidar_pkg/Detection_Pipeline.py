import time
import numpy as np
import open3d as o3d

import lidar_pkg.Downsampling as ds
import lidar_pkg.Segmentation as sg
import lidar_pkg.Clustering as cl
import lidar_pkg.BoundingBoxes as bb


def detection_pipeline(pcd, downsample_factor=0.25, iterations=100, tolerance=0.3, eps=0.4, min_points=5, debug=True, R=None, T=None):
    
    if R:
        rotation_matrix = pcd.get_rotation_matrix_from_xyz(R)
        pcd.rotate(rotation_matrix, center=(0, 0, 0))
    
    if T:
        pcd.translate(T)

    if debug:
        print(len(np.asarray(pcd.points)))
        o3d.visualization.draw_geometries([pcd])
    t = time.time()
    downsample_pcd = ds.downsample(pcd, downsample_factor)
    if debug:
        print("Downsample Time", time.time() - t)
        o3d.visualization.draw_geometries([downsample_pcd])

    t = time.time()
    inlier_pts, outlier_pts = sg.ransac(downsample_pcd, iterations=iterations, tolerance=tolerance)
    if debug:
        print("Segmentation Time", time.time() - t)
        o3d.visualization.draw_geometries([outlier_pts, inlier_pts])

    t = time.time()
    outlier_pts, labels = cl.dbscan(outlier_pts, eps=eps, min_points=min_points, print_progress=False, debug=debug)
    if debug:
        print("Clustering Time", time.time() - t)
        o3d.visualization.draw_geometries([outlier_pts, inlier_pts])

    t = time.time()
    bboxes = bb.oriented_bbox(outlier_pts, labels)

    #outlier_with_bboxes = [outlier_pts]
    outlier_with_bboxes = []
    outlier_with_bboxes.extend(bboxes)
    #outlier_with_bboxes.append(inlier_pts)
    
    if debug:
        print("Bounding Boxes Time", time.time() - t)
        o3d.visualization.draw_geometries(outlier_with_bboxes)

    return outlier_with_bboxes


if __name__ == "__main__":
    #pcl_file = "../Data/test_files/UDACITY/0000000008.pcd"
    pcl_file = "D:/Proyectos/LIDAR/GetLidarDataType_CH128x1_2024-03-15-22-14-52-937.pcd"
    a = detection_pipeline(pcl_file, debug=True)
