import struct
import numpy as np
import os


def read_xbin(file_path):
    """
    Reads a .xbin file and returns the point cloud data as a numpy array.
    Assumes each point is stored as 3 consecutive float values (x, y, z).
    """
    points = []
    try:
        with open(file_path, "rb") as f:
            while chunk := f.read(12):  # 12 bytes = 3 floats (x, y, z)
                if len(chunk) == 12:
                    x, y, z = struct.unpack("fff", chunk)
                    points.append([x, y, z])
        return np.array(points)
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return None


def write_pcd(points, output_path):
    """
    Writes point cloud data to a .pcd file.
    """
    try:
        num_points = points.shape[0]
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            f"WIDTH {num_points}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {num_points}\n"
            "DATA ascii\n"
        )
        with open(output_path, "w") as f:
            f.write(header)
            for point in points:
                f.write(f"{point[0]} {point[1]} {point[2]}\n")
        print(f"PCD file saved: {output_path}")
    except Exception as e:
        print(f"Error writing {output_path}: {e}")


def convert_xbin_to_pcd(xbin_file, pcd_file):
    """
    Converts a .xbin file to a .pcd file.
    """
    points = read_xbin(xbin_file)
    if points is not None:
        write_pcd(points, pcd_file)


# Example usage
if __name__ == "__main__":
    input_xbin = r"C:\Users\AMYES\OneDrive\Documents\GitHub\ComptuerEngineeringDesign\data\Speed Study\Flat 1rpm\20241121-14-38-13\10.xbin"  # Replace with your .xbin file path
    output_pcd = r"C:\Users\AMYES\OneDrive\Documents\GitHub\ComptuerEngineeringDesign\data\Speed Study\Flat 1rpm\pcdFiles\1rpmTest1\file10.pcd"  # Replace with your desired .pcd file path

    if os.path.exists(input_xbin):
        convert_xbin_to_pcd(input_xbin, output_pcd)
    else:
        print(f"Input file {input_xbin} does not exist.")
