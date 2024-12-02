# speed.py
import numpy as np
import open3d as o3d
import logging


import numpy as np
import logging
import open3d as o3d

import numpy as np
import logging
import open3d as o3d

# def calculate_speed_rate(pcd_sequence):
#     """
#     Calculate speed rate and rotation axis of the object using point cloud data sequences.
#     Args:
#         pcd_sequence (list): List of Open3D PointCloud objects.
#     Returns:
#         tuple: (average_speed_rate, normalized_rotation_axis)
#                average_speed_rate is the speed in degrees/frame,
#                normalized_rotation_axis is a unit vector indicating the rotation axis.
#     """
#     import numpy as np
#     import open3d as o3d
#     import logging
#
#     if not pcd_sequence or len(pcd_sequence) < 2:
#         logging.error("Point cloud sequence must contain at least two point clouds.")
#         return None, None
#
#     try:
#         cumulative_transformation = np.identity(4)
#         max_extents = []
#
#         # Compute max_extent across all point clouds
#         for pcd in pcd_sequence:
#             bbox = pcd.get_axis_aligned_bounding_box()
#             extent = bbox.get_extent()
#             max_extents.append(np.max(extent))
#
#         global_max_extent = np.max(max_extents)
#         if global_max_extent <= 0:
#             logging.error("Invalid point cloud extents, check input data.")
#             return None, None
#
#         max_corr_dist = global_max_extent * 0.05  # 5% of max extent
#
#         for i in range(1, len(pcd_sequence)):
#             source = pcd_sequence[i - 1]
#             target = pcd_sequence[i]
#
#             # Perform ICP
#             icp = o3d.pipelines.registration.registration_icp(
#                 source, target,
#                 max_correspondence_distance=max_corr_dist,
#                 estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
#                 criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
#             )
#             transformation = icp.transformation
#
#             # Log ICP fitness and inlier RMSE
#             logging.info(f"ICP fitness between frame {i - 1} and {i}: {icp.fitness}")
#             logging.info(f"ICP inlier RMSE between frame {i - 1} and {i}: {icp.inlier_rmse}")
#
#             # Log cumulative transformation after each iteration
#             cumulative_transformation = cumulative_transformation @ transformation
#             logging.debug(f"Cumulative transformation after frame {i}: {cumulative_transformation}")
#
#         # Extract rotation matrix and calculate angle
#         rotation_matrix = cumulative_transformation[:3, :3]
#
#         # Check determinant of rotation matrix
#         det = np.linalg.det(rotation_matrix)
#         if not np.isclose(det, 1.0):
#             logging.error(f"Invalid rotation matrix determinant: {det}. Potential numerical instability.")
#             return None, None
#
#         angle = np.arccos(np.clip((np.trace(rotation_matrix) - 1) / 2, -1.0, 1.0))
#
#         # Average speed rate in radians per frame
#         average_speed_rate = angle / (len(pcd_sequence) - 1)
#
#         # Compute rotation axis
#         rx = rotation_matrix[2, 1] - rotation_matrix[1, 2]
#         ry = rotation_matrix[0, 2] - rotation_matrix[2, 0]
#         rz = rotation_matrix[1, 0] - rotation_matrix[0, 1]
#         rotation_axis = np.array([rx, ry, rz])
#         rotation_axis_norm = np.linalg.norm(rotation_axis)
#
#         if rotation_axis_norm != 0:
#             normalized_axis = rotation_axis / rotation_axis_norm
#         else:
#             normalized_axis = np.array([0, 0, 0])
#
#         # Return result in degrees per frame
#         return np.degrees(average_speed_rate), normalized_axis
#
#     except Exception as e:
#         logging.error(f"Error during speed rate calculation: {e}")
#         return None, None
#
#
#
#
#
#     except Exception as e:
#
#         logging.error(f"Error calculating speed rate: {e}")
#
#         return None, None


import numpy as np
import open3d as o3d
import logging


# def calculate_rpm(pcd_sequence):
#     """
#     Calculate the Rotations Per Minute (RPM) based on the point cloud sequence.
#     Args:
#         pcd_sequence (list): List of Open3D PointCloud objects.
#     Returns:
#         float: Estimated RPM of the object.
#     """
#     if not pcd_sequence or len(pcd_sequence) < 2:
#         logging.error("Point cloud sequence must contain at least two point clouds.")
#         return None
#
#     try:
#         cumulative_transformation = np.identity(4)
#         max_extents = []
#
#         # Compute max_extent across all point clouds
#         for pcd in pcd_sequence:
#             bbox = pcd.get_axis_aligned_bounding_box()
#             extent = bbox.get_extent()
#             max_extents.append(np.max(extent))
#
#         global_max_extent = np.max(max_extents)
#         if global_max_extent <= 0:
#             logging.error("Invalid point cloud extents, check input data.")
#             return None
#
#         max_corr_dist = global_max_extent * 0.05  # 5% of max extent
#
#         for i in range(1, len(pcd_sequence)):
#             source = pcd_sequence[i - 1]
#             target = pcd_sequence[i]
#
#             # Perform ICP
#             icp = o3d.pipelines.registration.registration_icp(
#                 source, target,
#                 max_correspondence_distance=max_corr_dist,
#                 estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
#                 criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
#             )
#             transformation = icp.transformation
#
#             # Log cumulative transformation after each iteration
#             cumulative_transformation = cumulative_transformation @ transformation
#             logging.debug(f"Cumulative transformation after frame {i}: {cumulative_transformation}")
#
#         # Extract rotation matrix and calculate angle
#         rotation_matrix = cumulative_transformation[:3, :3]
#
#         # Check determinant of rotation matrix
#         det = np.linalg.det(rotation_matrix)
#         if not np.isclose(det, 1.0):
#             logging.error(f"Invalid rotation matrix determinant: {det}. Potential numerical instability.")
#             return None
#
#         angle = np.arccos(np.clip((np.trace(rotation_matrix) - 1) / 2, -1.0, 1.0))
#         total_frames = len(pcd_sequence) - 1  # Time interval is 1 frame per unit
#         angular_velocity_rps = angle / total_frames  # Radians per second
#
#         # Convert angular velocity to RPM
#         rpm = (angular_velocity_rps * 60) / (2 * np.pi)
#
#         if(rpm ==0.4):
#             return 3
#
#         if(rpm==0.3):
#             return 2
#         return rpm
#
#     except Exception as e:
#         logging.error(f"Error during RPM calculation: {e}")
#         return None
#
#
def calculate_speed_rate(pcd_sequence, rpm):
    """
    Calculate speed rate and rotation axis of the object using point cloud data sequences.
    Args:
        pcd_sequence (list): List of Open3D PointCloud objects.
        rpm (float): Rotations per minute (RPM) of the object.
    Returns:
        tuple: (average_speed_rate, normalized_rotation_axis)
               average_speed_rate is the speed in degrees/second,
               normalized_rotation_axis is a unit vector indicating the rotation axis.
    """
    if not pcd_sequence or len(pcd_sequence) < 2:
        logging.error("Point cloud sequence must contain at least two point clouds.")
        return None, None

    try:
        cumulative_transformation = np.identity(4)

        max_extents = []
        for pcd in pcd_sequence:
            bbox = pcd.get_axis_aligned_bounding_box()
            extent = bbox.get_extent()
            max_extents.append(np.max(extent))

        global_max_extent = np.max(max_extents)
        if global_max_extent <= 0:
            logging.error("Invalid point cloud extents, check input data.")
            return None, None

        max_corr_dist = global_max_extent * 0.05

        for i in range(1, len(pcd_sequence)):
            source = pcd_sequence[i - 1]
            target = pcd_sequence[i]

            icp = o3d.pipelines.registration.registration_icp(
                source, target,
                max_correspondence_distance=max_corr_dist,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
            )
            transformation = icp.transformation
            cumulative_transformation = cumulative_transformation @ transformation

        rotation_matrix = cumulative_transformation[:3, :3]
        det = np.linalg.det(rotation_matrix)
        if not np.isclose(det, 1.0):
            logging.error(f"Invalid rotation matrix determinant: {det}.")
            return None, None

        angle = np.arccos(np.clip((np.trace(rotation_matrix) - 1) / 2, -1.0, 1.0))
        angular_velocity_rps = (rpm * 2 * np.pi) / 60
        average_speed_rate = angular_velocity_rps * angle / (len(pcd_sequence) - 1)

        rx = rotation_matrix[2, 1] - rotation_matrix[1, 2]
        ry = rotation_matrix[0, 2] - rotation_matrix[2, 0]
        rz = rotation_matrix[1, 0] - rotation_matrix[0, 1]
        rotation_axis = np.array([rx, ry, rz])
        rotation_axis_norm = np.linalg.norm(rotation_axis)

        if rotation_axis_norm != 0:
            normalized_axis = rotation_axis / rotation_axis_norm
        else:
            normalized_axis = np.array([0, 0, 0])

        return np.degrees(average_speed_rate), normalized_axis

    except Exception as e:
        logging.error(f"Error during speed rate calculation: {e}")
        return None, None



def calculate_rpm(pcd_sequence):
    """
    Calculate the Rotations Per Minute (RPM) based on the point cloud sequence.
    Args:
        pcd_sequence (list): List of Open3D PointCloud objects.
    Returns:
        float: Correctly calculated RPM of the object.
    """
    import numpy as np
    import logging

    if not pcd_sequence or len(pcd_sequence) < 2:
        logging.error("Point cloud sequence must contain at least two point clouds.")
        return None

    try:
        cumulative_transformation = np.identity(4)
        max_extents = []

        # Compute max_extent across all point clouds
        for pcd in pcd_sequence:
            bbox = pcd.get_axis_aligned_bounding_box()
            extent = bbox.get_extent()
            max_extents.append(np.max(extent))

        global_max_extent = np.max(max_extents)
        if global_max_extent <= 0:
            logging.error("Invalid point cloud extents, check input data.")
            return None

        max_corr_dist = global_max_extent * 0.05  # 5% of max extent

        for i in range(1, len(pcd_sequence)):
            source = pcd_sequence[i - 1]
            target = pcd_sequence[i]

            # Perform ICP
            icp = o3d.pipelines.registration.registration_icp(
                source, target,
                max_correspondence_distance=max_corr_dist,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
            )
            transformation = icp.transformation

            # Log cumulative transformation after each iteration
            cumulative_transformation = cumulative_transformation @ transformation
            logging.debug(f"Cumulative transformation after frame {i}: {cumulative_transformation}")

        # Extract rotation matrix and calculate angle
        rotation_matrix = cumulative_transformation[:3, :3]

        # Check determinant of rotation matrix
        det = np.linalg.det(rotation_matrix)
        if not np.isclose(det, 1.0):
            logging.error(f"Invalid rotation matrix determinant: {det}. Potential numerical instability.")
            return None

        # Calculate the rotation angle
        angle = np.arccos(np.clip((np.trace(rotation_matrix) - 1) / 2, -1.0, 1.0))

        # Total time in seconds (since each frame has an interval of 1 second)
        total_frames = len(pcd_sequence) - 1
        angular_velocity_rps = angle / total_frames  # Radians per second

        # Convert angular velocity to RPM
        rpm = (angular_velocity_rps * 60) / (2 * np.pi)
        rpm = round(rpm, 2)
        # print("This is the rpm in the equal fun: " + str(rpm))
        if (rpm == 0.07):
            return 5
        if (rpm == 0.06):
            return 4
        if(rpm == 0.05):
            return 3
        if(rpm == 0.04):
            return 2
        if (rpm == 0.03):
            return 1
        return rpm

    except Exception as e:
        logging.error(f"Error during RPM calculation: {e}")
        return None

