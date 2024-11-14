import open3d as o3d
import numpy as np

# Given data (current dimensions of the bounding box)
current_width = 0.97151
current_height = 0.77428
current_depth = 0.48616

# Desired dimensions
desired_width = 18.5
desired_height = 8
desired_depth = 1.5

# Step 1: Calculate the scaling factors for each dimension
scaling_factors = np.array([
    desired_width / current_width,
    desired_height / current_height,
    desired_depth / current_depth
])

# Load the point cloud
clean_pcd = o3d.io.read_point_cloud("C:\\xt-pcds\\20241114-10-07-40_Binary.pcd")
if clean_pcd.is_empty():
    print("The point cloud is empty.")
    exit()

# Convert to numpy array for checking invalid points
points = np.asarray(clean_pcd.points)

# Check for NaN or Inf values and remove them if necessary
if np.any(np.isnan(points)) or np.any(np.isinf(points)):
    print("The point cloud contains invalid points.")
    clean_pcd = clean_pcd.select_by_index(np.where(np.isfinite(points).all(axis=1))[0])

# Check the number of points
num_points = len(clean_pcd.points)
print(f"Number of points in the point cloud: {num_points}")

# Perform plane segmentation if there are enough points
if num_points >= 3:
    plane_model, inliers = clean_pcd.segment_plane(
        distance_threshold=0.005, ransac_n=3, num_iterations=1000
    )
    print("Plane model:", plane_model)

    # Extract the inliers
    inlier_cloud = clean_pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([0, 1.0, 0])  # Color inliers green

    # Manually scale the inlier points
    inlier_points = np.asarray(inlier_cloud.points)
    scaled_points = inlier_points * scaling_factors  # Apply scaling factors
    inlier_cloud.points = o3d.utility.Vector3dVector(scaled_points)

    # Compute the axis-aligned bounding box of the scaled inliers
    aabb = inlier_cloud.get_axis_aligned_bounding_box()
    scaled_dimensions = aabb.get_extent()

    print(f"Scaled Dimensions (Width, Height, Depth): {scaled_dimensions}")

    # Visualize the inlier point cloud
    o3d.visualization.draw_geometries([inlier_cloud])
else:
    print("Point cloud is too sparse for plane segmentation.")
