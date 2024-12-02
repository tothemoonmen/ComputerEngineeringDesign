# motion.py
import numpy as np
import matplotlib.pyplot as plt
import logging

def is_moving(pcd_sequence):
    """

    Determine if the object in the sequence is moving.

    """

    try:

        # Compute the max_extent across all point clouds

        max_extents = []

        for pcd in pcd_sequence:
            bbox = pcd.get_axis_aligned_bounding_box()

            extent = bbox.get_extent()

            max_extent = np.max(extent)

            max_extents.append(max_extent)

        global_max_extent = np.max(max_extents)

        threshold = global_max_extent * 0.001  # Adjust threshold based on extent

        for i in range(1, len(pcd_sequence)):

            prev_pcd = pcd_sequence[i - 1]

            curr_pcd = pcd_sequence[i]

            # Compute distances between point clouds

            distances = prev_pcd.compute_point_cloud_distance(curr_pcd)

            mean_distance = np.mean(distances)

            logging.info(f"Mean distance between frame {i - 1} and {i}: {mean_distance}")

            if mean_distance > threshold:
                return True

        return False

    except Exception as e:

        logging.error(f"Error determining motion: {e}")

        return None

