# Define command constants (substitute the actual byte structure if necessary)
COMMAND_IMAGE = b'\xFB\x01'           # Command for sending one frame of image data
COMMAND_START = b'\xFB\x01'           # Start measurement
COMMAND_STOP = b'\xFB\x02'            # Stop measurement
COMMAND_SET_IP = b'\xFB\x03'          # Set IP address, subnet mask, network management IP
COMMAND_DEVICE_INFO = b'\xFB\x04'     # Request device information
COMMAND_CONFIG_INFO = b'\xFB\x05'     # Request configuration information
COMMAND_SET_FILTER = b'\xFB\x06'      # Set filter type and parameters
COMMAND_SET_INTEGRATION_TIME = b'\xFB\x08'  # Set integration time parameters
COMMAND_MIN_SIGNAL_AMPLITUDE = b'\xFB\x09'  # Set minimum signal amplitude
COMMAND_SET_HDR = b'\xFB\x0A'         # Set HDR
COMMAND_RESET = b'\xFB\x0D' + b'XINTAN'  # Reset radar, fixed string "XINTAN" needed
COMMAND_SET_FREQUENCY = b'\xFB\x34'   # Set modulation frequency
COMMAND_SET_ROI = b'\xFB\x33'         # Set Region of Interest (ROI)
COMMAND_TRACE_OUTPUT = b'\xFB\xD1'    # Trace output for logging

import socket
import serial

def send_command(connection, command, additional_data=b''):
    """ Send command to LiDAR with optional additional data. """
    full_command = command + additional_data
    connection.write(full_command)
    response = connection.read(4096)  # Adjust buffer size as needed
    print(response.decode('utf-8'))  # Assuming response is in text; modify as necessary

# Serial example (for USB or UART)
def serial_connection():
    ser = serial.Serial(port='COM3', baudrate=115200, timeout=1)  # Adjust port and baudrate
    return ser


# Connect to the LiDAR (choose serial or TCP)
connection = serial_connection()  # or tcp_connection()

# Send the "Start measurement" command
send_command(connection, COMMAND_START)



# Request device information
send_command(connection, COMMAND_DEVICE_INFO)

# Set HDR
hdr_type = b'\x01'  # example HDR type
send_command(connection, COMMAND_SET_HDR, hdr_type)

# Reset the radar
send_command(connection, COMMAND_RESET)

# Close connection after use
connection.close()
