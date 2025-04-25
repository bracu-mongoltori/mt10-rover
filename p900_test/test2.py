import serial
import time

# Set up the serial connection (adjust the port and baud rate)
ser = serial.Serial('/dev/ttyUSB0', 57600)  # Replace with your port
i = 0
while True:
    str =f"hello, serial- {i} \n"
    ser.write(str.encode())
    print(str)
    i+=1
      # Send the message as bytes
  # Wait for 1 second before sending the next message
