import numpy as np
import open3d as o3d
import numpy as np
import open3d as o3d

import preprocess

# def get_dem(pcd_input):
#
#     # Step 1: Calculate the scaling factors for each dimension
#     scaling_factors = np.array([12,
#                                 1.57,
#                               7.5])
#
#
#     # clean_pcd = preprocess.load_and_preprocess_pcd(pcd_input)
#     clean_pcd = pcd_input
#     # clean_pcd = o3d.io.read_point_cloud(file)  # Test 1 without curtain
#     if clean_pcd.is_empty():
#         print("The point cloud is empty.")
#
#     # points = np.asarray(clean_pcd.points)
#
#     # Check the number of points
#     num_points = len(clean_pcd.points)
#
#     # Only segment if there are enough points
#     if num_points >= 3:
#         # plane_model, inliers = clean_pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
#         # # Example: Reduce the distance threshold
#         plane_model, inliers = clean_pcd.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=1000)
#
#         print("Plane model:", plane_model)
#     else:
#         print("Point cloud is too sparse for plane segmentation.")
#
#     # Extract the inliers and outliers
#     inlier_cloud = clean_pcd.select_by_index(inliers, invert=True)
#
#     # Color the inlier cloud as green (the object being measured)
#     inlier_cloud.paint_uniform_color([0, 1.0, 0])  # Green for inliers
#
#     # Compute the axis-aligned bounding box
#     aabb = clean_pcd.get_axis_aligned_bounding_box()
#
#     # Get the extent (dimensions) of the bounding box
#     dimensions = aabb.get_extent()
#     scaled_dimensions = dimensions * scaling_factors
#
#     print(f"Dimensions (Width, Height, Depth): {scaled_dimensions}")
#     # print(f"Dimensions (Width, Height, Depth): {dimensions}")
#
#     # # Visualize the point clouds
#     # o3d.visualization.draw_geometries([inlier_cloud])
#
#     return scaled_dimensions


# Step 1: Calculate the scaling factors for each dimension
scaling_factors = np.array([12,
                            1.57,
                            7.5])


def get_dem(clean_pcd):
    if clean_pcd.is_empty():
        print("The point cloud is empty.")

    points = np.asarray(clean_pcd.points)

    # Check for NaN or Inf values
    if np.any(np.isnan(points)) or np.any(np.isinf(points)):
        print("The point cloud contains invalid points.")
        # Remove invalid points
        clean_pcd = clean_pcd.select_by_index(np.where(np.isfinite(points).all(axis=1))[0])
    # Check the number of points
    num_points = len(clean_pcd.points)
    print(f"Number of points in the point cloud: {num_points}")

    # Only segment if there are enough points
    if num_points >= 3:
        # plane_model, inliers = clean_pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
        # # Example: Reduce the distance threshold
        plane_model, inliers = clean_pcd.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=1000)

        print("Plane model:", plane_model)
    else:
        print("Point cloud is too sparse for plane segmentation.")

    # Extract the inliers and outliers
    inlier_cloud = clean_pcd.select_by_index(inliers, invert=True)

    # Color the inlier cloud as green (the object being measured)
    inlier_cloud.paint_uniform_color([0, 1.0, 0])  # Green for inliers

    # Compute the axis-aligned bounding box
    aabb = clean_pcd.get_axis_aligned_bounding_box()

    # Get the extent (dimensions) of the bounding box
    dimensions = aabb.get_extent()
    scaled_dimensions = dimensions * scaling_factors

    print(f"Dimensions (Width, Height, Depth): {scaled_dimensions}")
    # print(f"Dimensions (Width, Height, Depth): {dimensions}")
