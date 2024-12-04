import sys
import time
import vimba
import cv2
from vimba import *

def capture_image():
    try:
        with Vimba.get_instance() as vimba:
            cams = vimba.get_all_cameras()
            with cams[0] as cam:
                frame = cam.get_frame()
                frame.convert_pixel_format(PixelFormat.Mono8)
                cv2.imwrite('frame.jpg', frame.as_opencv_image())
    except Exception as e:
        print(f"An error occurred: {e}")


