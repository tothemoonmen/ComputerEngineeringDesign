# main.py
import os
import glob
import open3d as o3d
import numpy as np
import logging



from preprocess import load_and_preprocess_pcd
from dimensions import calculate_dimensions
from orientation import determine_orientation
from motion import is_moving
from speed import calculate_speed_rate

logging.basicConfig(level=logging.INFO)

def main():
    # Get the dataset directory from user input or config
    # dataset_dir = input("Enter the path to the dataset directory: ").strip()
    dataset_dir = "C:\\Users\\sahra\\PycharmProjects\\ComputerEngineeringDesign\\data\\Moving_PCD_Data\\Motion_PCD_Data\\Moving_a"
    # Get list of .pcd files in the directory
    pcd_files = sorted(glob.glob(os.path.join(dataset_dir, "*.pcd")))

    if not pcd_files:
        print("No .pcd files found in the directory.")
        return

    # Determine the type of dataset based on number of frames
    num_frames = len(pcd_files)
    print(f"Number of frames in the dataset: {num_frames}")

    # Load and preprocess all point clouds
    pcd_sequence = []
    for file_path in pcd_files:
        logging.info(f"Loading file: {file_path}")
        pcd = load_and_preprocess_pcd(file_path)
        if pcd is not None:
            pcd_sequence.append(pcd)
            logging.info(f"Loaded and preprocessed {file_path}")
        else:
            logging.warning(f"Failed to load or preprocess {file_path}")

    # If only one frame, calculate dimensions and orientation
    if num_frames == 1:
        pcd = pcd_sequence[0]
        # Scaling factors can be provided or calculated
        scaling_factors = None  # Or set to actual scaling factors if known

        dimensions = calculate_dimensions(pcd, scaling_factors)
        if dimensions is not None:
            print(f"Dimensions (Width, Height, Depth): {dimensions}")

        orientation_vector = determine_orientation(pcd)
        if orientation_vector is not None:
            print(f"Orientation vector: {orientation_vector}")

        # Determine if object is moving (with only one frame, assume not moving)
        moving = False
        print(f"Is object moving: {moving}")

    else:
        # Multiple frames, check if object is moving
        moving = is_moving(pcd_sequence)
        print(f"Is object moving: {moving}")

        # Calculate speed rate if moving
        if moving:
            # Assume time intervals are uniform and delta_t = 1 second between frames
            time_intervals = np.arange(num_frames)
            speed_rate, rotation_axis = calculate_speed_rate(pcd_sequence, time_intervals)
            print(f"Average speed rate: {speed_rate:.2f} degrees per second")
            RPM = speed_rate / 360 * 60  # Convert to RPM
            print(f"Estimated rotation rate: {RPM:.2f} RPM")
            print(f"Estimated rotation axis: {rotation_axis}")

        # For each frame, calculate dimensions and orientation (if needed)
        for idx, pcd in enumerate(pcd_sequence):
            print(f"\nProcessing frame {idx+1}/{num_frames}")
            dimensions = calculate_dimensions(pcd)
            if dimensions is not None:
                print(f"Dimensions (Width, Height, Depth): {dimensions}")

            orientation_vector = determine_orientation(pcd)
            if orientation_vector is not None:
                print(f"Orientation vector: {orientation_vector}")

if __name__ == "__main__":
    main()
