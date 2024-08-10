import serial
import time

while 1:
    port = input("Enter COM port or 'exit' to Exit: ")
    
    if port == 'exit':
        exit(0)

    try:
        print(f"Attempting to connect to {port}")
        ser = serial.Serial(port=port, baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
        print(f"Connected to {port}")
        break

    except Exception as e:
        print(f"Failed to connect to {port}. Please ensure the port is correct and try again.")
        print("Error: ")
        print(e)
        print()

while 1:
    data = input("Enter data to send or 'exit' to Exit: ")
    # data = "testing 101\r"
    if data == 'exit':
        ser.close()
        exit(0)

    data += '\r'
    ser.write(data.encode('utf-8'))
