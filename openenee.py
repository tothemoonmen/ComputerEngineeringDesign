import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import lidar

# Given data (current dimensions)
current_width = 0.97151
current_height = 0.77428
current_depth = 0.48616

# Desired dimensions
desired_width = 18.5
desired_height = 8
desired_depth = 1.5

# Step 1: Calculate the scaling factors for each dimension
scaling_factors = np.array([desired_width / current_width,
                            desired_height / current_height,
                            desired_depth / current_depth])

# Load the point cloud
clean_pcd = o3d.io.read_point_cloud("C:\\Users\\sahra\\PycharmProjects\\ComputerEngineeringDesign\\data\\rectangleHorizontalRotating\\20241113-10-25-10_Binary.pcd")  # Test 1 without curtain
if clean_pcd.is_empty():
    print("The point cloud is empty.")
    exit()

# Convert points to numpy array
points = np.asarray(clean_pcd.points)

# Check for NaN or Inf values and remove invalid points
if np.any(np.isnan(points)) or np.any(np.isinf(points)):
    print("The point cloud contains invalid points.")
    valid_indices = np.where(np.isfinite(points).all(axis=1))[0]
    clean_pcd = clean_pcd.select_by_index(valid_indices)

# Check the number of points
num_points = len(clean_pcd.points)
print(f"Number of points in the point cloud: {num_points}")

# Only segment if there are enough points
if num_points >= 3:
    # Segment the plane with adjusted parameters
    # plane_model, inliers = clean_pcd.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=1000)
    plane_model, inliers = clean_pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)


    print("Plane model:", plane_model)


    # Separate the inliers and outliers
    inlier_cloud = clean_pcd.select_by_index(inliers, invert=False)  # Inliers (points on the plane)
    outlier_cloud = clean_pcd.select_by_index(inliers, invert=True)  # Outliers (remaining points)

    # Assign green color to inliers
    num_inliers = len(inlier_cloud.points)
    inlier_colors = np.tile([0, 1.0, 0], (num_inliers, 1))  # Green
    inlier_cloud.colors = o3d.utility.Vector3dVector(inlier_colors)

    # Ensure outliers have their original color (or assign a default color)
    if outlier_cloud.colors:
        print("Outliers already have colors.")
    else:
        num_outliers = len(outlier_cloud.points)
        outlier_colors = np.tile([0.5, 0.5, 0.5], (num_outliers, 1))  # Gray
        outlier_cloud.colors = o3d.utility.Vector3dVector(outlier_colors)

    # Visualize the point cloud with separate colors
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    # Compute the axis-aligned bounding box
    if clean_pcd.has_points():
        aabb = clean_pcd.get_axis_aligned_bounding_box()
        dimensions = aabb.get_extent()
        scaled_dimensions = dimensions * scaling_factors

        print(f"Dimensions (Width, Height, Depth): {scaled_dimensions}")
        print(f"Original Dimensions (Width, Height, Depth): {dimensions}")
    else:
        print("Bounding box could not be calculated due to an empty point cloud.")

    # Visualize the point clouds
    if not inlier_cloud.is_empty():
        o3d.visualization.draw_geometries([inlier_cloud])
    else:
        print("No points to visualize.")
else:
    print("Point cloud is too sparse for plane segmentation.")

