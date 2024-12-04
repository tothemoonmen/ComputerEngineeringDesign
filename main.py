# main.py
import os
import glob
import open3d as o3d
import numpy as np
import logging
import re
import cam
import dimensions
import speed
import visualization
from preprocess import load_and_preprocess_pcd
from dimensions import get_dem
from orientation import determine_orientation
from motion import is_moving
from speed import calculate_speed_rate
import cv2
from vimba import *


def main():

    cam.capture_image()

    # visualization.visualize_pcl_file(r"C:\Users\AMYES\OneDrive\Documents\GitHub\ComptuerEngineeringDesign\data\Turning2rpm\20241201-18-36-07_Binary.pcd")
    # get_dem(r"C:\Users\AMYES\OneDrive\Documents\GitHub\ComptuerEngineeringDesign\data\Turning2rpm\20241201-18-36-07_Binary.pcd")
    # logging.basicConfig(level=logging.INFO)
    #
    # # dataset_dir = r"C:\Users\AMYES\OneDrive\Documents\GitHub\ComptuerEngineeringDesign\data\turningat1RPM"
    # # dim = dimensions.get_dem(r"C:\Users\AMYES\OneDrive\Documents\GitHub\ComptuerEngineeringDesign\data\turningat1RPM\20241201-16-59-08_Binary.pcd")
    # # Define dataset directory
    #
    #
    # dataset_dir = r"C:\Users\AMYES\OneDrive\Documents\GitHub\ComptuerEngineeringDesign\data\Turning2rpm"
    # dim = get_dem(r"C:\Users\AMYES\OneDrive\Documents\GitHub\ComptuerEngineeringDesign\data\Turning2rpm\20241201-18-35-49_Binary.pcd")
    # #
    # # dataset_dir = r"C:\Users\AMYES\OneDrive\Documents\GitHub\ComptuerEngineeringDesign\data\turning3rpm"
    # # dim = get_dem(
    # #     r"C:\Users\AMYES\OneDrive\Documents\GitHub\ComptuerEngineeringDesign\data\turning3rpm\20241201-18-48-19_Binary.pcd")
    #
    # frame_rate = 10  # frames per second (adjust if different)
    # # Get list of .pcd files in the directory
    # pcd_files = sorted(
    #     glob.glob(os.path.join(dataset_dir, "*.pcd")),
    #     key=lambda x: int(re.search(r'\d+', os.path.basename(x)).group())
    # )
    #
    # # Print sorted list for debugging
    # print(pcd_files)
    #
    # if not pcd_files:
    #     print("No .pcd files found in the directory.")
    #
    #     return
    #
    # # Determine the number of frames
    #
    # num_frames = len(pcd_files)
    #
    # print("\n---")
    #
    # print("### Data Preprocessing\n")
    #
    # print(f"Number of frames in the dataset: {num_frames}\n")
    #
    # # Load and preprocess all point clouds
    #
    # pcd_sequence = []
    # counter = 0
    #
    # for file_path in pcd_files:
    #
    #     logging.info(f"Loading file: {file_path}")
    #
    #     pcd = load_and_preprocess_pcd(file_path)
    #
    #     if pcd is not None:
    #
    #         pcd_sequence.append(pcd)
    #
    #         logging.info(f"Loaded and preprocessed {file_path}")
    #
    #     else:
    #
    #         logging.warning(f"Failed to load or preprocess {file_path}")
    #
    # # Check if any point clouds were loaded
    #
    # if not pcd_sequence:
    #     print("No valid point clouds were loaded.")
    #
    #     return
    #
    # # Use the first point cloud for dimension calculation
    #
    # first_pcd = pcd_sequence[0]
    #
    # # Calculate dimensions and orientation for the first frame
    #
    # # dimensions = calculate_dimensions(first_pcd)
    #
    # if dim is not None:
    #
    #     width, height, depth = dim
    #
    #     print("---")
    #
    #     print("### Dimension Calculation\n")
    #
    #     print(f"Calculated Dimensions (Width × Height × Depth): {width:.2f} cm × {height:.2f} cm × {depth:.2f} cm\n")
    #
    # else:
    #
    #     print("Failed to calculate dimensions.")
    #
    # orientation_vector = determine_orientation(first_pcd)
    #
    # if orientation_vector is not None:
    #
    #     print("---")
    #
    #     print("### Orientation Determination\n")
    #
    #     print(
    #         f"Orientation Vector: [{orientation_vector[0]:.2f}, {orientation_vector[1]:.2f}, {orientation_vector[2]:.2f}]\n")
    #
    # else:
    #
    #     print("Failed to determine orientation.")
    #
    # # Check if the object is moving
    #
    # moving = is_moving(pcd_sequence)
    #
    # print("---")
    #
    # print("### Motion Detection\n")
    #
    # print(f"Is the object moving? {'Yes' if moving else 'No'}\n")
    #
    # if moving:
    #     rpm = speed.calculate_rpm(pcd_sequence)
    #
    #     speed_rate, rotation_axis = calculate_speed_rate(pcd_sequence, rpm)
    #
    #     if speed_rate is not None:
    #
    #         print("---")
    #
    #         print("### Speed and Rotation Axis Calculation\n")
    #
    #         # print(f"Average Speed Rate: {speed_rate:.2f} degrees per second\n")
    #
    #         print(f"Estimated Rotation Rate: {rpm:.2f} RPM\n")
    #
    #         print(
    #             f"Estimated Rotation Axis: [{rotation_axis[0]:.2f}, {rotation_axis[1]:.2f}, {rotation_axis[2]:.2f}]\n")
    #
    #     else:
    #
    #         print("Failed to calculate speed rate.")
    #
    # else:
    #
    #     print("Object is not moving.")
    #
    # # Optionally, predict orientation after 10 revolutions
    #
    # if moving and speed_rate is not None and rotation_axis is not None:
    #
    #     num_revolutions = 10
    #
    #     total_rotation_angle = num_revolutions * 360  # degrees
    #
    #     if speed_rate != 0:
    #
    #         time_needed = total_rotation_angle / speed_rate
    #
    #         final_orientation_angle = (speed_rate / 360)
    #
    #         print("---")
    #
    #         print("### Orientation Prediction\n")
    #
    #         print(f"Time needed for {num_revolutions} revolutions: {time_needed:.2f} seconds\n")
    #
    #         # print(
    #         #     f"Predicted Orientation after {num_revolutions} revolutions: {final_orientation_angle:.2f} degrees around axis [{rotation_axis[0]:.2f}, {rotation_axis[1]:.2f}, {rotation_axis[2]:.2f}]\n")
    #
    #     else:
    #
    #         print("Cannot predict future orientation because speed rate is zero.")
    #
    # else:
    #
    #     print("Cannot predict future orientation without motion data.")


if __name__ == "__main__":
    main()
