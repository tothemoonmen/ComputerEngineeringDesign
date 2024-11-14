import sys
import time
from vimba import *

def capture_image():
    try:
        # Initialize Vimba
        with Vimba() as vimba:
            # Get the first available camera
            camera = vimba.get_all_cameras()[0]

            # Open the camera
            camera.open()

            # Start the camera acquisition
            camera.start_frame_grabber()

            # Grab a frame
            frame = camera.get_frame()

            # Wait for the frame to be ready
            frame.wait_for_frame()

            # Get the image data from the frame
            image_data = frame.as_numpy()

            # Save the image to a file (e.g., PNG or JPEG format)
            from PIL import Image
            image = Image.fromarray(image_data)
            image.save("captured_image.png")

            print("Image captured and saved as 'captured_image.png'.")

            # Stop the camera acquisition
            camera.stop_frame_grabber()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    capture_image()
