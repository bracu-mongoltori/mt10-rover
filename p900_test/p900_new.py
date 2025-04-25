import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from rclpy.timer import Timer

class SerialCommunicator(Node):
    def __init__(self):
        super().__init__('serial_communicator')
        
        # Setup serial communication
        self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)  # Adjust port and baudrate as needed

        # Timer to read from serial every 1 second
        self.timer = self.create_timer(0.00001, self.read_from_serial)

        # Publisher to send data to another node
        self.publisher = self.create_publisher(String, 'read_telemetry', 1)

        # Subscriber to receive data and send it to the serial port
        self.create_subscription(String, 'write_telemetry', self.write_to_serial, 1)

    def read_from_serial(self):
        """ Read data from the serial device and publish it if available """
        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode('utf-8').strip()  # Read and decode serial data
            self.get_logger().info(f'Read from serial: {data}')
            
            # Create ROS2 message
            msg = String()
            msg.data = data

            # Publish the message
            self.publisher.publish(msg)

    def write_to_serial(self, msg: String):
        """ Write received message to the serial device """
        self.get_logger().info(f'Writing to serial: {msg.data}')
        msg.data += "\n"
        self.ser.write(msg.data.encode('utf-8'))  # Write data to serial

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommunicator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()  # Close the serial port when shutting down
        rclpy.shutdown()

if __name__ == '__main__':
    main()

