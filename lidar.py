# Define command constants (substitute the actual byte structure if necessary)
COMMAND_IMAGE = b'\xFB\x01'  # Command for sending one frame of image data
COMMAND_START = b'\xFB\x01'  # Start measurement
COMMAND_STOP = b'\xFB\x02'  # Stop measurement
COMMAND_SET_IP = b'\xFB\x03'  # Set IP address, subnet mask, network management IP
COMMAND_DEVICE_INFO = b'\xFB\x04'  # Request device information
COMMAND_CONFIG_INFO = b'\xFB\x05'  # Request configuration information
COMMAND_SET_FILTER = b'\xFB\x06'  # Set filter type and parameters
COMMAND_SET_INTEGRATION_TIME = b'\xFB\x08'  # Set integration time parameters
COMMAND_MIN_SIGNAL_AMPLITUDE = b'\xFB\x09'  # Set minimum signal amplitude
COMMAND_SET_HDR = b'\xFB\x0A'  # Set HDR
COMMAND_RESET = b'\xFB\x0D' + b'XINTAN'  # Reset radar, fixed string "XINTAN" needed
COMMAND_SET_FREQUENCY = b'\xFB\x34'  # Set modulation frequency
COMMAND_SET_ROI = b'\xFB\x33'  # Set Region of Interest (ROI)
COMMAND_TRACE_OUTPUT = b'\xFB\xD1'  # Trace output for logging
# Constants for the radar communication
TCP_PORT = 7787  # Default TCP port for commands
UDP_PORT = 7687  # Default UDP port for image data
RADAR_IP = '192.168.0.101'  # Lidar's IP address
BUFFER_SIZE = 4096  # Adjust buffer size for UDP data

import serial
import struct
import open3d as o3d


def serial_connection(port='COM4', baudrate=115200, timeout=1):
    """ Initialize a serial connection. """
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        return ser
    except serial.SerialException as e:
        print(f"Failed to open serial connection: {e}")
        return None


def send_command(connection, command, additional_data=b''):
    """ Send command to LiDAR with optional additional data and return the response. """
    try:
        full_command = command + additional_data
        connection.write(full_command)
        response = connection.read(4096)  # Adjust buffer size as needed
        return response
    except Exception as e:
        print(f"Error sending command: {e}")
        return None


def parse_raw_data(data):
    """ Parse binary data into a list of (x, y, z) points.

    This loop iterates over the data in steps of 12 bytes (because each 3D point is represented by 3 float32 values,
    and each float32 is 4 bytes). The loop starts at index 0 and increments by 12 on each iteration until it reaches
    the end of the data.
    """
    points = []
    try:
        for i in range(0, len(data), 12):  # Assuming 12 bytes per point (float32 x, y, z)
            x, y, z = struct.unpack('fff', data[i:i + 12])
            points.append((x, y, z))
    except struct.error as e:
        print(f"Error parsing raw data: {e}")
    return points


def save_as_pcd(points, filename="output.pcd"):
    """ Save a list of points (x, y, z) as a .pcd file. """
    if not points:
        print("No points to save.")
        return

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(filename, point_cloud)
    print(f"Saved point cloud to {filename}")


def get_image():
    """ Get image data from the LiDAR and save it as a .pcd file. """
    connection = serial_connection()
    if not connection:
        return  # Exit if connection failed

    try:
        print("Sending command to retrieve image...")
        response = send_command(connection, COMMAND_IMAGE)

        if response:
            print(f"Received {len(response)} bytes of data.")
            points = parse_raw_data(response)
            print(f"Parsed {len(points)} points from the data.")
            save_as_pcd(points, "lidar_image.pcd")
        else:
            print("No response received from the LiDAR.")
    finally:
        connection.close()
        print("Serial connection closed.")


import serial


def set_integration_time(group_time, grayscale_time):
    connection = serial_connection()
    if not connection:
        return  # Exit if connection failed
    """
    Set the integration time for the LiDAR.

    Parameters:
        connection (serial.Serial): The serial connection to the LiDAR.
        group_time (int): Integration time for the 8 4 group (range: 0-255).
        grayscale_time (int): Integration time for grayscale (range: 0-255).

    Returns:
        bytes: The response from the LiDAR.
    """
    try:
        # Validate inputs
        if not (0 <= group_time <= 255):
            raise ValueError("Group integration time must be between 0 and 255.")
        if not (0 <= grayscale_time <= 255):
            raise ValueError("Grayscale integration time must be between 0 and 255.")

        # Construct the command
        command_id = COMMAND_SET_INTEGRATION_TIME  # Replace with the actual command ID for setting integration time
        parameters = bytes([group_time, grayscale_time])

        # Send command
        response = send_command(connection, command_id, additional_data=parameters)
        return response
    except Exception as e:
        print(f"Error setting integration time: {e}")
        return None


def set_filters():
    connection = serial_connection()
    # Define the filter parameters
    kalman_factor = 500  # Example: Moderate Kalman factor
    kalman_threshold = 1500  # Example: High Kalman threshold
    flyingspot_threshold = 50  # Example: Flyingspot detection enabled with a low threshold

    if not connection:
        return  # Exit if connection failed
    """
    Set the filters for the LiDAR.

    Parameters:
        kalman_factor (int): Kalman factor (0-1000, 0 means disabled).
        kalman_threshold (int): Kalman threshold (0-2000, 0 means disabled).
        flyingspot_threshold (int): Flyingspot threshold (0 means disabled).

    Returns:
        bytes: The response from the LiDAR.
    """
    try:
        # Validate inputs
        if not (0 <= kalman_factor <= 1000):
            raise ValueError("Kalman factor must be between 0 and 1000.")
        if not (0 <= kalman_threshold <= 2000):
            raise ValueError("Kalman threshold must be between 0 and 2000.")
        if not (0 <= flyingspot_threshold <= 65535):  # Flyingspot threshold is 2 bytes, range is 0-65535
            raise ValueError("Flyingspot threshold must be between 0 and 65535.")

        # Convert parameters to 2-byte little-endian format
        kalman_factor_bytes = kalman_factor.to_bytes(2, byteorder='little')
        kalman_threshold_bytes = kalman_threshold.to_bytes(2, byteorder='little')
        flyingspot_threshold_bytes = flyingspot_threshold.to_bytes(2, byteorder='little')

        # Construct the command
        command = COMMAND_SET_FILTER  # This is b'\xFB\x06' as defined
        parameters = kalman_factor_bytes + kalman_threshold_bytes + flyingspot_threshold_bytes

        # Send the command
        response = send_command(connection, command, additional_data=parameters)
        if response:
            print(f"Response from LiDAR: {response}")
        else:
            print("No response received or an error occurred.")
        return response
    except Exception as e:
        print(f"Error setting filters: {e}")
        return None
