# main.py
import os
import glob
import time

import open3d as o3d
import numpy as np
import logging
import re
import cam
import dimensions
import lidar
import preprocess
import speed
import visualization
from preprocess import load_and_preprocess_pcd
from dimensions import get_dem
from orientation import determine_orientation
import motion

import cv2
from vimba import *


def main():


    # Path to folder containing image sequence to calculate RPM and is Moving
    folder_path_1RPM = r"C:\Users\AMYES\OneDrive\Documents\GitHub\ComptuerEngineeringDesign\Project_Final_Data\Camera_1rpm_Take1_10SecIntervals"
    folder_path_3RPM = r"C:\Users\AMYES\OneDrive\Documents\GitHub\ComptuerEngineeringDesign\Project_Final_Data\3RPM_8Secs"


    images = motion.load_images_from_folder(folder_path_3RPM)  #Motion for 3 RPM
    is_moving = motion.detect_overall_motion(images)
    no_motion_folder = r"C:\Users\AMYES\OneDrive\Documents\GitHub\ComptuerEngineeringDesign\Project_Final_Data\No_Motion"

    # Output result
    if is_moving:
        print("Object is moving across the sequence.")
    else:
        print("Object is not moving across the sequence.")

    rpm = speed.calculate_rpm_image(folder_path_3RPM, 6) #calculate 3RPM folder
    print(f" The date set {folder_path_3RPM} has and RPM of: {rpm}")

    rpm = speed.calculate_rpm_image(folder_path_1RPM, 6) #calculate 3RPM folder
    print(f" The date set {folder_path_1RPM} has and RPM of: {rpm}")

    images_not_moving_folder = motion.load_images_from_folder(no_motion_folder)
    images_not_moving= motion.detect_overall_motion(images_not_moving_folder)

    if images_not_moving:
        print("Object is moving across the sequence.")

    else:
        print("Object is not moving across the sequence.")


    # visualization.visualize_pcl_file(preprocess.load_and_preprocess_pcd(r"C:\xt-pcds\20241210-14-51-24_Binary.pcd"))
    file = r"C:\xt-pcds\20241211-09-00-22_Binary.pcd"
    pcd_input = o3d.io.read_point_cloud(file)
    get_dem(pcd_input)
    visualization.visualize_pcl_file(pcd_input)





if __name__ == "__main__":
    main()
