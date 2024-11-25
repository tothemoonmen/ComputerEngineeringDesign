# motion.py
import numpy as np

def is_moving(pcd_sequence, threshold=0.01):
    """
    Compare consecutive point clouds to see if the object has moved.
    Return True if moving, False otherwise.
    """
    if len(pcd_sequence) < 2:
        return False

    for i in range(len(pcd_sequence) - 1):
        pcd1 = pcd_sequence[i]
        pcd2 = pcd_sequence[i+1]
        # Compute the difference between point clouds
        points1 = np.asarray(pcd1.points)
        points2 = np.asarray(pcd2.points)
        # Align point clouds if necessary (not implemented here)
        if points1.shape == points2.shape:
            diff = np.linalg.norm(points1 - points2, axis=1)
            mean_diff = np.mean(diff)
            if mean_diff > threshold:
                return True
        else:
            return True  # Assume moving if shapes differ
    return False
