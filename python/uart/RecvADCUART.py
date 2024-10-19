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
def read_serial_data():
    received_data = []
    if ser.in_waiting > 0:  # Check if there is incoming data
        try:
            while ser.in_waiting != 0:
                received_data.append(ser.readline().decode('utf-8'))
            return received_data
        except Exception as e:
            print(f"Error reading from serial port: {e}")
    return None

# Main loop to send and receive data

while True:
    if ser.in_waiting > 0:
        incoming_data = read_serial_data()
        last_message_time = time.time()

    for data in incoming_data:
        if data != None:
            print(f"{data}")
    
    if incoming_data:
        if time.time() - last_message_time > 0.1:
            print("Time since last message: ", time.time() - last_message_time)

    incoming_data = []