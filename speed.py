# speed.py
import numpy as np
import open3d as o3d
import logging
import os
import cv2



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



# def calculate_rpm(pcd_sequence):
#     """
#     Calculate the Rotations Per Minute (RPM) based on the point cloud sequence.
#     Args:
#         pcd_sequence (list): List of Open3D PointCloud objects.
#     Returns:
#         float: Correctly calculated RPM of the object.
#     """
#     import numpy as np
#     import logging
#
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
#         # Calculate the rotation angle
#         angle = np.arccos(np.clip((np.trace(rotation_matrix) - 1) / 2, -1.0, 1.0))
#
#         # Total time in seconds (since each frame has an interval of 1 second)
#         total_frames = len(pcd_sequence) - 1
#         angular_velocity_rps = angle / total_frames  # Radians per second
#
#         # Convert angular velocity to RPM
#         rpm = (angular_velocity_rps * 60) / (2 * np.pi)
#         rpm = round(rpm, 2)
#
#         return rpm
#
#     except Exception as e:
#         logging.error(f"Error during RPM calculation: {e}")
#         return None




def calculate_rpm_image(image_folder, time_interval):
    """
    Calculate the RPM of a moving object using images.

    Args:
        image_folder (str): Path to the folder containing images.
        time_interval (float): Time interval between consecutive images (in seconds).

    Returns:
        float: Calculated RPM.
    """
    try:
        # Load images from the folder
        image_files = sorted(
            [os.path.join(image_folder, f) for f in os.listdir(image_folder) if f.endswith(('.jpg', '.png'))]
        )
        if not image_files:
            raise Exception("No images found in the specified folder.")

        feature_positions = []

        for image_file in image_files:
            # Read the image
            image = cv2.imread(image_file)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Apply preprocessing (adjust thresholds based on your application)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            _, binary = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)

            # Detect contours
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Select the largest contour or a specific feature
            largest_contour = max(contours, key=cv2.contourArea)
            moments = cv2.moments(largest_contour)

            # Calculate the centroid of the contour (feature position)
            if moments["m00"] != 0:
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])
                feature_positions.append((cx, cy))

        # Calculate RPM
        revolutions = 0
        last_position = None

        for position in feature_positions:
            if last_position is not None:
                # Check if the feature completes a revolution (e.g., crosses a specific axis)
                if position[1] > last_position[1]:  # Adjust condition as needed
                    revolutions += 1
            last_position = position

        total_time = time_interval * len(image_files)  # Total time of captured images
        rpm = (revolutions / total_time) * 60  # RPM calculation

        return rpm

    except Exception as e:
        print(f"An error occurred: {e}")
        return None

