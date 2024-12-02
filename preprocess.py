#------------------------ preprocess.py - -----------------------
# Imports

import os

import glob

import open3d as o3d

import numpy as np

import logging


def load_and_preprocess_pcd(file_path):
    """

    Load and preprocess a point cloud data file (.pcd).

    """

    try:

        # Load point cloud

        pcd = o3d.io.read_point_cloud(file_path)

        # Check if the point cloud is empty

        if len(pcd.points) == 0:
            logging.error(f"Point cloud {file_path} is empty.")

            return None

        # Convert to numpy array

        points = np.asarray(pcd.points)

        # Remove NaN and infinite values

        finite_indices = np.all(np.isfinite(points), axis=1)

        points = points[finite_indices]

        pcd.points = o3d.utility.Vector3dVector(points)

        # Remove statistical outliers

        if len(points) > 0:

            _, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

            pcd = pcd.select_by_index(ind)

        else:

            logging.error(f"Point cloud {file_path} has no finite points after removing invalid values.")

            return None

        # Downsample the point cloud

        voxel_size = estimate_voxel_size(pcd)

        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

        # Estimate normals

        pcd.estimate_normals(

            search_param=o3d.geometry.KDTreeSearchParamHybrid(

                radius=voxel_size * 2, max_nn=30)

        )

        return pcd



    except Exception as e:

        logging.error(f"Error loading point cloud {file_path}: {e}")

        return None


def estimate_voxel_size(pcd):
    """

    Estimate an appropriate voxel size for downsampling.

    """

    bbox = pcd.get_axis_aligned_bounding_box()

    extent = bbox.get_extent()

    max_extent = np.max(extent)

    voxel_size = max_extent * 0.01  # Set voxel size to 1% of max extent

    if voxel_size == 0:
        voxel_size = 0.01  # Default to 0.01 if extent is zero

    logging.info(f"Estimated voxel_size: {voxel_size}")

    return voxel_size