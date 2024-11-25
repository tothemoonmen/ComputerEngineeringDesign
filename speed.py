# speed.py
import numpy as np
import open3d as o3d

def compute_rotation_axis_angle(transformation):
    """
    Extract rotation axis and angle from the transformation matrix.
    """
    rotation_matrix = transformation[:3, :3]
    angle = np.arccos((np.trace(rotation_matrix) - 1) / 2)
    angle_deg = np.degrees(angle)

    if np.sin(angle) != 0:
        rx = (rotation_matrix[2,1] - rotation_matrix[1,2]) / (2 * np.sin(angle))
        ry = (rotation_matrix[0,2] - rotation_matrix[2,0]) / (2 * np.sin(angle))
        rz = (rotation_matrix[1,0] - rotation_matrix[0,1]) / (2 * np.sin(angle))
        rotation_axis = np.array([rx, ry, rz])
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
    else:
        rotation_axis = np.array([0, 0, 0])

    return angle_deg, rotation_axis

import logging

def compute_rotation_angle(pcd1, pcd2):
    logging.info("Starting ICP registration between two point clouds.")
    threshold = 0.05  # Adjust threshold as needed
    trans_init = np.identity(4)

    try:
        reg_p2p = o3d.pipelines.registration.registration_icp(
            pcd1, pcd2, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())

        transformation = reg_p2p.transformation
        logging.info("ICP registration completed successfully.")
    except Exception as e:
        logging.error(f"ICP registration failed: {e}")
        return 0.0, np.array([0, 0, 0])

    angle, rotation_axis = compute_rotation_axis_angle(transformation)
    return angle, rotation_axis

def calculate_speed_rate(pcd_sequence, time_intervals):
    """
    Given a sequence of point clouds and time intervals, calculate the average speed rate and rotation axis.
    """
    if len(pcd_sequence) < 2:
        return 0.0, None

    total_angle = 0.0
    rotation_axes = []

    for i in range(len(pcd_sequence) - 1):
        pcd1 = pcd_sequence[i]
        pcd2 = pcd_sequence[i+1]
        time_diff = time_intervals[i+1] - time_intervals[i]

        angle, rotation_axis = compute_rotation_angle(pcd1, pcd2)
        total_angle += angle
        rotation_axes.append(rotation_axis)

    average_angle = total_angle / (len(pcd_sequence) - 1)
    total_time = time_intervals[-1] - time_intervals[0]
    average_speed = total_angle / total_time  # degrees per second

    # Compute average rotation axis
    rotation_axes = np.array(rotation_axes)
    average_axis = np.mean(rotation_axes, axis=0)
    average_axis = average_axis / np.linalg.norm(average_axis)

    return average_speed, average_axis
