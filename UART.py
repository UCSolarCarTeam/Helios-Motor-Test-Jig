import serial
import time

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

def read_adc_data(time):
    start_time = time.time()
    while time.time() - start_time < time:
        incoming_data = read_serial_data()
        if incoming_data:
            data.append(incoming_data)

# Main loop to send and receive data
while True:
    payload = input("Enter data to send or 'exit' to Exit: ")
    
    if payload.lower() == 'exit':
        ser.close()
        exit(0)

    if payload.lower() == 'adc':
        # Read the ADC data for specified time
        read_time = input("Enter the time in seconds to read ADC data: ")
        read_adc_data(read_time + 1)

    # Send data to the serial port
    payload += '\r'  # Adding carriage return
    ser.write(payload.encode('utf-8'))
    print(f"Sent: {payload.strip()}")
    print()

    if payload.lower() == 'adc':
        # Read the ADC data for specified time
    
    else:
    # Try to read data after sending
        time.sleep(1)  # Optional delay to ensure the receiving end gets time
        incoming_data = read_serial_data()

        if incoming_data:
            data.append(incoming_data)
