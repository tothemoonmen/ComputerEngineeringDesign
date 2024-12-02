import numpy as np
import open3d as o3d
import numpy as np
import open3d as o3d


# Desired dimensions
# desired_width = 18.5
# desired_height = 8


# # Step 1: Calculate the scaling factors for each dimension
# scaling_factors = np.array([
#     desired_width / current_width,
#     desired_height / current_height,
#     desired_depth / current_depth
# ])
#
#
# def clean_point_cloud(clean_pcd):
#     """
#     Preprocess the point cloud to remove invalid rows and print points for debugging.
#
#     Args:
#         clean_pcd (o3d.geometry.PointCloud): Input point cloud.
#
#     Returns:
#         o3d.geometry.PointCloud: Cleaned point cloud.
#     """
#     points = np.asarray(clean_pcd.points)
#
#     # Print original points
#     print("Original Points:")
#     print(points)
#
#     # Filter out invalid points (zeros, NaN, Inf, extreme values)
#     valid_indices = np.where(
#         np.isfinite(points).all(axis=1) &  # Remove NaN/Inf
#         ~np.all(points == 0, axis=1) &  # Remove rows of zeros
#         (np.abs(points) < 1e6).all(axis=1)  # Remove extreme values
#     )[0]
#
#     # Print valid points
#     valid_points = points[valid_indices]
#     print("\nFiltered Points (Valid Rows Only):")
#     print(valid_points)
#
#     # Select valid points
#     clean_pcd = clean_pcd.select_by_index(valid_indices)
#
#     return clean_pcd


# def calculate_dimensions(clean_pcd, scaling_factors=None):
#     """
#     Calculate the dimensions of a point cloud by extracting inliers via plane segmentation
#     and computing the bounding box of the scaled inlier points.
#
#     Args:
#         clean_pcd (o3d.geometry.PointCloud): Input point cloud.
#         scaling_factors (list or np.array): Scaling factors for the dimensions. Defaults to [1, 1, 1].
#
#     Returns:
#         np.array: Dimensions of the bounding box (Width, Height, Depth).
#     """
#     # Clean the point cloud (you might already have this logic elsewhere)
#     points = np.asarray(clean_pcd.points)
#     if len(points) == 0:
#         print("Point cloud is empty.")
#         return None
#
#     # Perform plane segmentation
#     plane_model, inliers = clean_pcd.segment_plane(
#         distance_threshold=0.005, ransac_n=3, num_iterations=1000
#     )
#     if len(inliers) == 0:
#         print("No plane could be detected in the point cloud.")
#         return None
#
#     inlier_cloud = clean_pcd.select_by_index(inliers)
#
#     # Apply default scaling factors if none are provided
#     if scaling_factors is None or len(scaling_factors) != 3:
#         print("Invalid or missing scaling factors. Using default [1, 1, 1].")
#         scaling_factors = np.array([1, 1, 1])
#
#     # Scale the inlier points
#     inlier_points = np.asarray(inlier_cloud.points)
#     scaled_points = inlier_points * scaling_factors
#     inlier_cloud.points = o3d.utility.Vector3dVector(scaled_points)
#
#     # Compute axis-aligned bounding box
#     aabb = inlier_cloud.get_axis_aligned_bounding_box()
#     dimensions = aabb.get_extent()
#
#     print(f"Dimensions (Width, Height, Depth): {dimensions}")
#     return dimensions




# def get_rectangle_dimensions(point_cloud_path: str):
#     """
#     Compute the dimensions of a rectangle from its point cloud.
#
#     Args:
#         point_cloud_path (str): Path to the point cloud file (e.g., .ply, .pcd).
#
#     Returns:
#         tuple: Width, height, and optional depth of the rectangle.
#     """
#     # Load the point cloud
#     point_cloud = o3d.io.read_point_cloud(point_cloud_path)
#
#     if not point_cloud.has_points():
#         raise ValueError("The provided point cloud has no points!")
#
#     # Compute the axis-aligned bounding box
#     aabb = point_cloud.get_axis_aligned_bounding_box()
#
#     # Dimensions of the rectangle
#     bounding_box_extent = aabb.get_extent()  # [width, height, depth]
#
#     width, height, depth = bounding_box_extent
#
#     print(f"Width: {width}")
#     print(f"Height: {height}")
#     print(f"Depth: {depth}")
#
#     return width, height, depth


def get_dem(file):
    # Given data (current dimensions)
    current_width = 0.831
    current_height = 0.256
    current_depth = 0.199

    # Desired dimensions
    desired_width = 10
    desired_height = 4.5
    desired_depth = 1.5

    # Step 1: Calculate the scaling factors for each dimension
    scaling_factors = np.array([desired_width / current_width,
                                desired_height / current_height,
                                desired_depth / current_depth])

    clean_pcd = o3d.io.read_point_cloud(file)  # Test 1 without curtain
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
    print(f"Dimensions (Width, Height, Depth): {dimensions}")

    return scaled_dimensions


