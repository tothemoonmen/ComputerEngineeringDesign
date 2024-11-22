

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

original_width = 0.42236234
original_height = 0.44203399
original_depth = 0.19203022

# Desired dimensions in meters (converted from cm)
desired_width = 0.185
desired_height = 0.08
desired_depth = 0.015

# Calculating scaling factors

# Step 1: Calculate the scaling factors for each dimension
scaling_factors = np.array([desired_width / original_width,
                            desired_height / original_height,
                            desired_depth / original_depth])
# Load the point cloud
clean_pcd = o3d.io.read_point_cloud("C:\\xt-pcds\\20241117-14-41-26_Binary.pcd")# Test 1 without curtain Can use curtain are not just set the mesurment distance where the object is only in frame
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

print(f"Dimensions in meters (Width, Height, Depth): {scaled_dimensions}")
print(f"Dimensions (Width, Height, Depth): {dimensions}")



# Visualize the point clouds
o3d.visualization.draw_geometries([inlier_cloud])


# 18.5 cm width
#  8 cm height
# 1.5 cm depth
