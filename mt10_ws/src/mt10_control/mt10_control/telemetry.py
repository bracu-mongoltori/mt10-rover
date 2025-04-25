import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64
from sbg_driver.msg import SbgGpsPos
import serial
from rclpy.timer import Timer

class SerialCommunicator(Node):
    def __init__(self):
        super().__init__('serial_communicator')
        
        # Setup serial communication
        self.ser = serial.Serial('/dev/ttyUSB0', 56700, timeout=1)  # Adjust port and baudrate as needed

        # Timer to read from serial every 1 second
        global count
        count=0
        self.timer = self.create_timer(0.001, self.write_to_serial)

        # Publisher to send data to another node
        #self.publisher = self.create_publisher(String, '/minipc_read_telemetry', 1)

        # Subscriber to receive data and send it to the serial port
        #self.create_subscription(String, '/minipc_write_telemetry', self.write_to_serial, 1)
        
        # Subscriber for sending host machine topic data
        #self.create_subscription(Float64, "/witmotion_eular/yaw", self.yaw_callback, 10)
        
        #self.create_subscription(SbgGpsPos, "/sbg/gps_pos", self.gps_callback, 10)
        
        #self.create_subscription(String, "/status", self.status_callback, 10)
        
        

    # def read_from_serial(self):
    #     """ Read data from the serial device and publish it if available """
    #     if self.ser.in_waiting > 0:
    #         data = self.ser.readline().decode('utf-8').strip()  # Read and decode serial data
    #         self.get_logger().info(f'Read from serial: {data}')
            
    #         # Create ROS2 message
    #         msg = String()
    #         msg.data = data

    #         # Publish the message
    #         self.publisher.publish(msg)

    def write_to_serial(self, msg):
        """ Write received message to the serial device """
        self.get_logger().info(f'Writing to serial: {msg}')
        count+=1
        msg.data = '{count}'
        msg += "\n"
        self.ser.write(msg.encode('utf-8'))  # Write data to serial
        
    

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

