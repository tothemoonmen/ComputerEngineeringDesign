# orientation.py
import numpy as np

def determine_orientation(pcd):
    """
    Use PCA to determine the principal axes and return the orientation vector.
    """
    points = np.asarray(pcd.points)
    if points.shape[0] < 3:
        print("Not enough points to determine orientation.")
        return None

    mean = np.mean(points, axis=0)
    centered_points = points - mean
    cov = np.cov(centered_points, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    # Sort eigenvalues and eigenvectors
    idx = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]
    # Return the orientation as the direction of the principal axis
    orientation_vector = eigenvectors[:, 0]
    return orientation_vector
