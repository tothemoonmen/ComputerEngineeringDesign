# motion.py
import numpy as np
import matplotlib.pyplot as plt
import logging
import cv2
import numpy as np
import os


def load_images_from_folder(folder):
    """
    Load all image files from a specified folder.
    """
    images = []
    for filename in sorted(os.listdir(folder)):
        img_path = os.path.join(folder, filename)
        img = cv2.imread(img_path)
        if img is not None:
            images.append(img)
    return images



def detect_overall_motion(images, threshold=30, motion_pixel_ratio=0.02):
    """
    Detect if there is motion across the entire sequence of images.

    Args:
    - images (list): List of images as numpy arrays.
    - threshold (int): Pixel difference threshold for motion detection.
    - motion_pixel_ratio (float): Proportion of pixels that need to change to confirm motion.

    Returns:
    - is_moving (bool): True if motion is detected in the sequence, False otherwise.
    """
    total_motion_ratio = 0  # Sum of motion ratios across all frames
    frame_count = len(images) - 1  # Total number of frame comparisons

    for i in range(1, len(images)):
        # Convert both frames to grayscale
        prev_gray = cv2.cvtColor(images[i - 1], cv2.COLOR_BGR2GRAY)
        curr_gray = cv2.cvtColor(images[i], cv2.COLOR_BGR2GRAY)

        # Compute absolute difference
        diff = cv2.absdiff(prev_gray, curr_gray)

        # Threshold the difference
        _, thresh = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)

        # Calculate the ratio of changed pixels
        non_zero_count = np.count_nonzero(thresh)
        total_pixels = thresh.size
        motion_ratio = non_zero_count / total_pixels

        # Accumulate the motion ratio
        total_motion_ratio += motion_ratio

    # Average motion ratio across all frames
    average_motion_ratio = total_motion_ratio / frame_count if frame_count > 0 else 0

    # Determine overall motion
    is_moving = average_motion_ratio > motion_pixel_ratio
    print(f"Average Motion Ratio: {average_motion_ratio:.4f}, Motion Detected: {is_moving}")

    return is_moving

# def is_moving(pcd_sequence):
#     """
#
#     Determine if the object in the sequence is moving.
#
#     """
#
#     try:
#
#         # Compute the max_extent across all point clouds
#
#         max_extents = []
#
#         for pcd in pcd_sequence:
#             bbox = pcd.get_axis_aligned_bounding_box()
#
#             extent = bbox.get_extent()
#
#             max_extent = np.max(extent)
#
#             max_extents.append(max_extent)
#
#         global_max_extent = np.max(max_extents)
#
#         threshold = global_max_extent * 0.001  # Adjust threshold based on extent
#
#         for i in range(1, len(pcd_sequence)):
#
#             prev_pcd = pcd_sequence[i - 1]
#
#             curr_pcd = pcd_sequence[i]
#
#             # Compute distances between point clouds
#
#             distances = prev_pcd.compute_point_cloud_distance(curr_pcd)
#
#             mean_distance = np.mean(distances)
#
#             logging.info(f"Mean distance between frame {i - 1} and {i}: {mean_distance}")
#
#             if mean_distance > threshold:
#                 return True
#
#         return False
#
#     except Exception as e:
#
#         logging.error(f"Error determining motion: {e}")
#
#         return None

