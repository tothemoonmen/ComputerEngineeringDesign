import open3d as o3d
import numpy as np
import torch
import torchvision
from torchvision.io import read_image
from torchvision.utils import draw_bounding_boxes

import preprocess


# def visualize_pcl_file(pcd_input, save_as_image=False, image_path="point_cloud.png"):
#     """
#     Visualize a point cloud with preprocessing and optional image saving.
#
#     Parameters:
#     pcd_input (open3d.geometry.PointCloud): Input point cloud data.
#     save_as_image (bool): If True, saves the visualization as an image.
#     image_path (str): Path to save the image if save_as_image is True.
#     """
#
#     # Step 1: Convert to NumPy array
#     points = np.asarray(pcd_input.points)
#
#     # Step 2: Filter invalid points
#     valid_indices = np.where(
#         np.isfinite(points).all(axis=1) & ~np.all(points == 0, axis=1)
#     )[0]
#     filtered_points = points[valid_indices]
#
#     print(f"Valid points after filtering Vaild points: {len(filtered_points)}")
#
#     # Step 3: Check for unique points
#     unique_points = np.unique(points, axis=0)
#     print(f"Unique points: {len(unique_points)}")
#
#     # Step 4: Center and scale for visualization
#     center = np.mean(unique_points, axis=0)
#     scaled_points = (unique_points - center) / np.max(
#         np.linalg.norm(unique_points - center, axis=1)
#     )
#
#     # Create scaled PointCloud
#     scaled_pcd = o3d.geometry.PointCloud()
#     scaled_pcd.points = o3d.utility.Vector3dVector(scaled_points)
#
#     # Add colors (optional)
#     scaled_pcd.paint_uniform_color([0.1, 0.7, 0.8])  # Light blue
#
#
#
#     # Create an axis-aligned bounding box from the scaled point cloud
#     aabb = scaled_pcd.get_axis_aligned_bounding_box()
#     aabb.color = (1, 0, 0)  # Set color to red for visibility
#
#     o3d.visualization.draw_geometries([scaled_pcd], window_name="Scaled Point Cloud")
#
#
#     # Define new, smaller dimensions for the bounding box
#
#     # Create the bounding box as you've already defined
#     min_bound = [-0.5, -0.07, 0]  # Minimum corner (x_min, y_min, z_min)
#     max_bound = [0.4, 0.5, 0.3]  # Maximum corner (x_max, y_max, z_max)
#     aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
#     aabb.color = (1, 0, 0)  # Set color to red for visibility
#
#     # Crop the point cloud using the AABB (this keeps only the points within the box)
#     cropped_pcd = scaled_pcd.crop(aabb)


    # points = np.asarray(cropped_pcd.points)
    # print(f"After Using bounding box there are: {len(points)}")

    # # Visualize the point cloud and the bounding box
    # o3d.visualization.draw_geometries([scaled_pcd, obb], window_name="Point Cloud with Oriented Bounding Box")

    # Visualize the point cloud and the bounding box
    # o3d.visualization.draw_geometries([scaled_pcd], window_name="Scaled Point Cloud")



    #
    #
    # # Step 5: Visualize or save
    # if save_as_image:
    #     # Create visualization
    #     vis = o3d.visualization.Visualizer()
    #     vis.create_window(visible=False)
    #     vis.add_geometry(scaled_pcd)
    #     vis.get_render_option().background_color = np.array([0, 0, 0])  # Black background
    #     vis.get_render_option().point_size = 2.0  # Larger points
    #
    #     # Configure camera (optional: adjust to your scene)
    #     ctr = vis.get_view_control()
    #     ctr.set_zoom(0.8)
    #     ctr.set_lookat([0, 0, 0])
    #     ctr.set_up([0, -1, 0])
    #
    #     # Capture image
    #     vis.poll_events()
    #     vis.update_renderer()
    #     vis.capture_screen_image(image_path)
    #     vis.destroy_window()
    #     print(f"Point cloud visualization saved as: {image_path}")



def draw_bounding_box():
    # read input image from your computer
    img = read_image(
        r"C:\Users\AMYES\OneDrive\Documents\GitHub\ComptuerEngineeringDesign\Camera_2rpm_Take1\image_20241206_102741_505052_1.jpg")

    # bounding box are xmin, ymin, xmax, ymax
    box = [550, 420, 1050, 650]
    box = torch.tensor(box)
    box = box.unsqueeze(0)

    # draw bounding box and fill color
    img = draw_bounding_boxes(img, box, width=5,
                              colors="green",
                              fill=True)

    # transform this image to PIL image
    img = torchvision.transforms.ToPILImage()(img)

    # display output
    img.show()

    import open3d as o3d
    import numpy as np

    def visualize_pcl_file(pcd_input_path, save_as_image=False, image_path="point_cloud.png"):
        """
        Visualize a point cloud with preprocessing and optional image saving.

        Parameters:
        pcd_input_path (str): Path to the input point cloud file.
        save_as_image (bool): If True, saves the visualization as an image.
        image_path (str): Path to save the image if save_as_image is True.
        """
        # Step 1: Load point cloud
        pcd = o3d.io.read_point_cloud(pcd_input_path)
        if pcd.is_empty():
            print("The point cloud is empty.")
            return

        # Step 2: Convert to NumPy array and filter invalid points
        points = np.asarray(pcd.points)
        valid_indices = np.where(
            np.isfinite(points).all(axis=1) & ~np.all(points == 0, axis=1)
        )[0]
        filtered_points = points[valid_indices]

        if len(filtered_points) == 0:
            print("No valid points found after filtering.")
            return

        print(f"Valid points after filtering: {len(filtered_points)}")

        # Step 3: Check for unique points
        unique_points = np.unique(filtered_points, axis=0)
        print(f"Unique points: {len(unique_points)}")

        # Step 4: Center and scale for visualization
        center = np.mean(unique_points, axis=0)
        scaled_points = (unique_points - center) / np.max(
            np.linalg.norm(unique_points - center, axis=1)
        )

        # Create a new PointCloud for visualization
        scaled_pcd = o3d.geometry.PointCloud()
        scaled_pcd.points = o3d.utility.Vector3dVector(scaled_points)

        # Add colors (optional)
        scaled_pcd.paint_uniform_color([0.1, 0.7, 0.8])  # Light blue

        # Step 5: Create axis-aligned bounding box for scaled point cloud
        aabb = scaled_pcd.get_axis_aligned_bounding_box()
        aabb.color = (1, 0, 0)  # Red for visibility

        # Step 6: Visualization
        o3d.visualization.draw_geometries(
            [scaled_pcd, aabb], window_name="Scaled Point Cloud"
        )


import open3d as o3d
import numpy as np

def visualize_pcl_file(pcd_input_path, save_as_image=False, image_path="point_cloud.png"):
    """
    Visualize a point cloud with preprocessing and optional image saving.

    Parameters:
    pcd_input_path (str): Path to the input point cloud file.
    save_as_image (bool): If True, saves the visualization as an image.
    image_path (str): Path to save the image if save_as_image is True.
    """
    # Step 1: Load point cloud
    pcd = pcd_input_path
    if pcd.is_empty():
        print("The point cloud is empty.")
        return

    # Step 2: Convert to NumPy array and filter invalid points
    points = np.asarray(pcd.points)
    valid_indices = np.where(
        np.isfinite(points).all(axis=1) & ~np.all(points == 0, axis=1)
    )[0]
    filtered_points = points[valid_indices]

    if len(filtered_points) == 0:
        print("No valid points found after filtering.")
        return

    print(f"Valid points after filtering: {len(filtered_points)}")

    # Step 3: Check for unique points
    unique_points = np.unique(filtered_points, axis=0)
    print(f"Unique points: {len(unique_points)}")

    # Step 4: Center and scale for visualization
    center = np.mean(unique_points, axis=0)
    scaled_points = (unique_points - center) / np.max(
        np.linalg.norm(unique_points - center, axis=1)
    )

    # Create a new PointCloud for visualization
    scaled_pcd = o3d.geometry.PointCloud()
    scaled_pcd.points = o3d.utility.Vector3dVector(scaled_points)

    # Add colors (optional)
    scaled_pcd.paint_uniform_color([0.1, 0.7, 0.8])  # Light blue

    # Create the bounding box as you've already defined
    min_bound = [-0.7, -0.22, 0]  # Minimum corner (x_min, y_min, z_min)
    max_bound = [0.6, 0.3, 0.3]  # Maximum corner (x_max, y_max, z_max)
    aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
    aabb.color = (1, 0, 0)  # Set color to red for visibility

    # Step 6: Visualization
    o3d.visualization.draw_geometries(
        [scaled_pcd, aabb], window_name="Scaled Point Cloud"
    )

