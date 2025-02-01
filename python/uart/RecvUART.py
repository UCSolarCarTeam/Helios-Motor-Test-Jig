# This script reads data from a serial port and prints it to the console
# Authored By: Dominic Choi

import serial
import time

incoming_data = []

# Initialize the serial connection
while True:
    port = input("Enter COM port or 'exit' to Exit: ").strip().replace(' ', '').upper()

    if port == 'EXIT':
        exit(0)

    try:
        print(f"Attempting to connect to {port}")
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=8,
            timeout=2,  # Set a timeout for reading
            stopbits=serial.STOPBITS_ONE
        )
        print(f"Connected to {port}")
        break

    except Exception as e:
        print(f"Failed to connect to {port}. Please ensure the port is correct and try again.")
        print("Error: ")
        print(e)
        exit(1)

# Function to read incoming UART data
def read_serial_data(ser):
    try:
        if ser.in_waiting > 0:  # Check if data is available
            data = ser.readline().decode('utf-8').strip()  # Read and decode a line
            return data  # Return the data
    except Exception as e:
        #print(f"Error reading data: {e}")
        print()
    return None

# Main loop to send and receive data
while True:
    # Read serial data and append it to the list if valid
    new_data = read_serial_data(ser)
    if new_data is not None:
        incoming_data.append(new_data)

    # Process and print all incoming data
    for data in incoming_data:
        print(f"{data}")

    # Clear the list after processing
    incoming_data.clear()