import serial

# Set up the serial connection (adjust the port and baud rate)
ser = serial.Serial('/dev/ttyUSB0', 57600)  # Use the correct port for your system

# Send the message
ser.write(b"Hello, Serial!")  # Send the message as bytes

# Close the serial connection
ser.close()

