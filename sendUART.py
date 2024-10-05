import serial
import time

data = []

# Initialize the serial connection
while True:
    port = input("Enter COM port or 'exit' to Exit: ")
    
    if port.lower() == 'exit':
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
    received_data = []
    if ser.in_waiting > 0:  # Check if there is incoming data
        try:
            while ser.in_waiting > 0:
                received_data.append(ser.readline().decode('utf-8').strip())
                print(f"{received_data}")
            return received_data
        except Exception as e:
            print(f"Error reading from serial port: {e}")
    return None

# Main loop to send and receive data
while True:
    data = input("Enter data to send or 'exit' to Exit: ")
    
    if data.lower() == 'exit':
        ser.close()
        exit(0)

    # Send data to the serial port
    data += '\r'  # Adding carriage return
    ser.write(data.encode('utf-8'))
    print(f"Sent: {data.strip()}")

    # Try to read data after sending
    time.sleep(1)  # Optional delay to ensure the receiving end gets time
    incoming_data = read_serial_data()

    if incoming_data:
        data.append(incoming_data)
