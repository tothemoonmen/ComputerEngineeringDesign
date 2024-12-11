#------------------------ preprocess.py - -----------------------
# Imports
import os
import glob

import open3d as o3d

import numpy as np

import logging


def load_and_preprocess_pcd(pcd_input):
    """
    Load and preprocess a point cloud data file (.pcd).
    """
    try:

        # Load point cloud

        print(f"There are this many points in the pcd file: {len(pcd_input.points)}")

        # Check if the point cloud is empty
        if len(pcd_input.points) == 0:
            logging.error(f"Point cloud {pcd_input} is empty.")
            return None

        # Step 1: Convert to NumPy array
        points = np.asarray(pcd_input.points)

        # Step 2: Filter invalid points
        valid_indices = np.where(
            np.isfinite(points).all(axis=1) & ~np.all(points == 0, axis=1)
        )[0]
        filtered_points = points[valid_indices]

        print(f"Valid points after filtering Vaild points: {len(filtered_points)}")


        x_min, x_max = 0, 1000
        y_min, y_max = 0, 1000
        z_min, z_max = -10, 10


        # Apply bounding box filtering
        bbox_indices = np.where(
            (points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
            (points[:, 1] >= y_min) & (points[:, 1] <= y_max) &
            (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        )[0]
        filtered_points_box = points[bbox_indices]

        if len(filtered_points_box) == 0:
            print("Warning: No points found within the bounding box!")

        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

        # Visualize point cloud and bounding box
        pcd_input.paint_uniform_color([1, 0, 0])  # Red for filtered points

        print(f"The amount of points after bounding box filtering: {len(points)}")

        # Remove statistical outliers
        if len(points) > 0:
            _, ind = pcd_input.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            filtered_points = filtered_points.select_by_index(ind)
        else:
            logging.error(f"Point cloud {pcd_input} has no points after bounding box filtering.")
            return None


        print(f"The amount of points after processing is {len(filtered_pcd.points)}")

        return filtered_pcd

    except Exception as e:
        logging.error(f"Error loading point cloud {pcd_input}: {e}")
        return None


# Helper function for voxel size estimation
def estimate_voxel_size(pcd):
    """
    Estimate an appropriate voxel size for downsampling.
    """
    bounding_box = pcd.get_axis_aligned_bounding_box()
    diagonal_length = np.linalg.norm(bounding_box.get_extent())
    return diagonal_length / 100  # Adjust the divisor as necessary for your dataset
