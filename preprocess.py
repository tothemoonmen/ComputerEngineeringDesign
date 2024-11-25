# preprocess.py
import open3d as o3d
import numpy as np

def load_and_preprocess_pcd(file_path):
    """
    Load the point cloud, check for NaNs/Infs, remove invalid points,
    and return the cleaned point cloud.
    """
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(file_path)
    if pcd.is_empty():
        print("The point cloud is empty.")
        return None

    # Convert points to numpy array
    points = np.asarray(pcd.points)

    # Check for NaN or Inf values and remove invalid points
    if np.any(np.isnan(points)) or np.any(np.isinf(points)):
        print("The point cloud contains invalid points.")
        valid_indices = np.where(np.isfinite(points).all(axis=1))[0]
        pcd = pcd.select_by_index(valid_indices)

    return pcd
