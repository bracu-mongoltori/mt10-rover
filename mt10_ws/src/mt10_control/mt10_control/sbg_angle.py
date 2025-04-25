import rclpy
from rclpy.node import Node
from sbg_driver.msg import SbgEkfEuler
import math

class YawLogger(Node):
    def __init__(self):
        super().__init__('yaw_logger')
        self.subscription = self.create_subscription(
            SbgEkfEuler,
            '/sbg/ekf_euler',
            self.yaw_callback,
            10)

    def yaw_callback(self, msg):
        yaw_deg = math.degrees(msg.angle.z)  # Yaw is stored in angle.z
        self.get_logger().info(f'Yaw: {yaw_deg:.2f}°')

def main(args=None):
    rclpy.init(args=args)
    node = YawLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
