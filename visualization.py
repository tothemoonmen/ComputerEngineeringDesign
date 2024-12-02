import open3d as o3d
import numpy as np

def visualize_pcl_file(file):
    pcd = o3d.io.read_point_cloud(file)
    # Convert to NumPy array
    points = np.asarray(pcd.points)

    # print(f"Total points loaded: {len(points)}")
    # print("Sample points:")
    # print(points)

    # Step 1: Total points
    print(f"Total points loaded: {len(points)}")


    # Step 2: Filter invalid points
    valid_indices = np.where(
        np.isfinite(points).all(axis=1) & ~np.all(points == 0, axis=1)
    )[0]
    filtered_points = points[valid_indices]
    print(f"Valid points after filtering: {len(filtered_points)}")

    # Step 3: Check for unique points
    unique_points = np.unique(filtered_points, axis=0)
    print(f"Unique points: {len(unique_points)}")

    # Step 4: Center and scale for visualization
    center = np.mean(unique_points, axis=0)
    scaled_points = (unique_points - center) / np.max(np.linalg.norm(unique_points - center, axis=1))
    scaled_pcd = o3d.geometry.PointCloud()
    scaled_pcd.points = o3d.utility.Vector3dVector(scaled_points)

    # Step 5: Visualize
    o3d.visualization.draw_geometries([scaled_pcd], window_name="Scaled Point Cloud")