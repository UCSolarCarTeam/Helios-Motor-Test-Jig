# Sends and receives data over UART using Python
# Created by: Dominic Choi

import serial
import time

import Profiles as pfl

p = pfl.Profiles("profiles-config/profiles.json")

data = []

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
        print()

# Function to read incoming UART data
def read_serial_data():
    received_data = ''
    if ser.in_waiting > 0:  # Check if there is incoming data
        try:
            while ser.in_waiting != 0:
                received_data += ser.readline().decode('utf-8')
            print(f"{received_data}")
            return received_data
        except Exception as e:
            print(f"Error reading from serial port: {e}")
    return None

# Main loop to send and receive data
while True:
    payload = input("Enter data to send or 'exit' to Exit: ")

    if payload.lower() == 'exit':
        ser.close()
        exit(0)

    elif payload.lower().split()[0] == 'profile':
        profile_option = payload.split()[1]
        if profile_option == 'list':
            print("Profiles:")
            for profile in p.get_profiles():
                print(profile.get("name"))
            print()

        elif profile_option == 'check':
            profile_name = payload.split()[2]
            commands = p.get_commands_by_profile_name(profile_name)
            if commands:
                print(f"Profile '{profile_name}' command sequence:")
                for command in commands:
                    print(command)
                print()
            else:
                print(f"Profile '{profile_name}' not found.")

        elif profile_option == 'set':
            profile_name = payload.split()[2]
            commands = p.get_commands_by_profile_name(profile_name)
            if commands:
                for command in commands:
                    command += '\r'
                    ser.write(command.encode('utf-8'))
                    print(f"Sent: {command}")
                    time.sleep(0.5)
                    incoming_data = read_serial_data()
                    
            else:
                print(f"Profile '{profile_name}' not found.")
        else:
            print("Invalid profile option.")

    else: 
        # Send data to the serial port
        payload += '\r'  # Adding carriage return
        ser.write(payload.encode('utf-8'))
        print(f"Sent: {payload.strip()}")
        print()
        
        # Try to read data after sending
        time.sleep(0.5)  # Optional delay to ensure the receiving end gets time
        incoming_data = read_serial_data()

        if incoming_data:
            data.append(incoming_data)
