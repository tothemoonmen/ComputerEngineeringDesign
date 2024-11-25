# dimensions.py
import open3d as o3d
import numpy as np

def calculate_dimensions(pcd, scaling_factors=None):
    """
    Compute the axis-aligned bounding box and return the dimensions.
    If scaling_factors are provided, scale the dimensions accordingly.
    """
    if pcd.has_points():
        aabb = pcd.get_axis_aligned_bounding_box()
        dimensions = aabb.get_extent()
        if scaling_factors is not None:
            scaled_dimensions = dimensions * scaling_factors
            return scaled_dimensions
        else:
            return dimensions
    else:
        print("Bounding box could not be calculated due to an empty point cloud.")
        return None
