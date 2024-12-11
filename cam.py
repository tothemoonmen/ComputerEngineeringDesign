import sys
import time
import vimba
import cv2
from vimba import *

import cam



import os
from datetime import datetime
import cv2
from vimba import Vimba, PixelFormat  # Adjust imports based on your Vimba SDK setup.

import os
import time
from datetime import datetime
import cv2
from vimba import Vimba, PixelFormat  # Adjust imports based on your Vimba SDK setup.


def capture_images_bgr8(num_images, save_dir, time_interval):
    """
    Capture multiple images in Bgr8 format and save them to a specified directory with a time interval.

    Args:
        num_images (int): Number of images to capture.
        save_dir (str): Directory to save the captured images.
        time_interval (float): Time interval between captures, in seconds.

    Raises:
        Exception: If there is an error during image capture or saving.
    """
    try:
        # Ensure the save directory exists
        os.makedirs(save_dir, exist_ok=True)

        # Connect to the Vimba system
        with Vimba.get_instance() as vimba:
            cams = vimba.get_all_cameras()
            if not cams:
                raise Exception("No cameras found!")

            # Open the first available camera
            with cams[0] as cam:
                for i in range(num_images):
                    frame = cam.get_frame()
                    frame.convert_pixel_format(PixelFormat.Bgr8)

                    # Generate a unique filename
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                    filename = f"image_{timestamp}_{i + 1}.jpg"
                    filepath = os.path.join(save_dir, filename)

                    # Save the frame as an image
                    cv2.imwrite(filepath, frame.as_opencv_image())
                    print(f"Saved: {filepath}")

                    # Wait for the specified time interval before capturing the next image
                    time.sleep(time_interval)

    except Exception as e:
        print(f"An error occurred: {e}")


def capture_image_mono():
    try:
        with Vimba.get_instance() as vimba:
            cams = vimba.get_all_cameras()
            with cams[0] as cam:
                frame = cam.get_frame()
                frame.convert_pixel_format(PixelFormat.Bgr8)
                cv2.imwrite('frame.jpg', frame.as_opencv_image())
    except Exception as e:
        print(f"An error occurred: {e}")


def set_pixel_format(pixel_format):
    cam.set_pixel_format(pixel_format)


def load_camera_settings():
    """
    Load settings into the camera from a specified settings file.

    :param camera: The camera object.
    :param settings_file: Path to the settings file.
    """
    try:
        with Vimba.get_instance() as vimba:
            cams = vimba.get_all_cameras()
            with cams[0] as cam:
                # Open the camera for configuration
                cam.save_settings("camreaSettings.xml", PersistType.All)

            # Apply settings file
            # camera.load_settings(settings_file, PersistType.All)
            # print("Settings loaded successfully.")


    except Exception as ex:
        print(f"An error occurred: {ex}")



def configure_camera(camera: Camera):
    """
    Configure camera settings for optimal clarity.
    :param camera: The camera object to configure.
    """
    try:
        with camera:
            print(f"Configuring camera {camera.get_name()} for optimal clarity...")

            # Adjust exposure
            if 'ExposureAuto' in camera.get_all_features_names():
                camera.get_feature_by_name('ExposureAuto').set('Off')  # Turn off auto mode
            if 'ExposureTime' in camera.get_all_features_names():
                camera.get_feature_by_name('ExposureTime').set(20000.0)  # Set in microseconds

            # Adjust gain
            if 'GainAuto' in camera.get_all_features_names():
                camera.get_feature_by_name('GainAuto').set('Off')  # Turn off auto gain
            if 'Gain' in camera.get_all_features_names():
                camera.get_feature_by_name('Gain').set(10.0)  # Adjust as needed

            # Adjust white balance
            if 'BalanceWhiteAuto' in camera.get_all_features_names():
                camera.get_feature_by_name('BalanceWhiteAuto').set('Continuous')

            # Adjust resolution and pixel format
            if 'Width' in camera.get_all_features_names():
                camera.get_feature_by_name('Width').set(1920)  # Adjust to your desired resolution
            if 'Height' in camera.get_all_features_names():
                camera.get_feature_by_name('Height').set(1080)
            if 'PixelFormat' in camera.get_all_features_names():
                camera.get_feature_by_name('PixelFormat').set('Mono8')  # or 'RGB8Packed'

            print("Camera configured successfully.")


    except Exception as ex:
        print(f"An unexpected error occurred: {ex}")



# capture_images_bgr8(10, "Project_Final_Data/No_Motion", 8)
