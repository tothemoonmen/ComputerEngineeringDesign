import logging

import numpy as np

def determine_orientation(pcd):

    """

    Determine the orientation of a point cloud using PCA.

    """

    try:

        points = np.asarray(pcd.points)

        # Center the points

        mean = np.mean(points, axis=0)

        centered_points = points - mean

        # Compute covariance matrix

        cov = np.cov(centered_points.T)

        # Compute eigenvalues and eigenvectors

        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        # The principal axis is the eigenvector corresponding to the largest eigenvalue

        principal_axis = eigenvectors[:, np.argmax(eigenvalues)]

        return principal_axis

    except Exception as e:

        logging.error(f"Error determining orientation: {e}")

        return None