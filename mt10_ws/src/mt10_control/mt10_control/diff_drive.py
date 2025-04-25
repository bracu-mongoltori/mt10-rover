import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial

class RoverBase(Node):

    def __init__(self):
        super().__init__("rover_base")
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('port', '/dev/ttyACM0')

        cmd_vel = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().string_value

        self.rover = self.create_subscription(Twist, cmd_vel, self.listener_callback, 1)
        self.rover_controller = serial.Serial(port, baudrate= 115200)

        self.get_logger().info(f"Rover base initialized with port, {port}, and subscription to {cmd_vel}")

    def listener_callback(self, msg):
        lin_vel = msg.linear.x
        ang_vel = msg.angular.z
#        print(type(lin_vel))
        data = f"<{lin_vel},{ang_vel}>"
        self.rover_controller.write(data.encode("UTF-8"))
        self.get_logger().info(f"current linear speed is {lin_vel} and angular speed is {ang_vel}")

def main(args=None):
    rclpy.init(args=args)

    rover = RoverBase()

    rclpy.spin(rover)
    rover.rover_controller.write(f"{0.0} {0.0}".encode("UTF-8"))
    rclpy.shutdown()

if __name__ == "__main__":
    main()
