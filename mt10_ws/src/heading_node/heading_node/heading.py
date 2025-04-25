import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64
import math

def quaternion_to_euler(q_w, q_x, q_y, q_z):
    # Roll (x-axis rotation)
    roll = math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))
    
    # Pitch (y-axis rotation)
    pitch = math.asin(2 * (q_w * q_y - q_z * q_x))
    
    # Yaw (z-axis rotation)
    yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
    
    return roll, pitch, yaw

class QuaternionToEulerNode(Node):
    def __init__(self):
        super().__init__('quaternion_to_euler')
        self.subscription = self.create_subscription(
            Quaternion,
            '/orientation',
            self.orientation_callback,
            10
        )
        self.yaw = self.create_publisher(Float64, "/witmotion_eular/yaw", 10)
        self.get_logger().info('Quaternion to Euler node has started.')

        self.yaw_value = Float64()

    def orientation_callback(self, msg):
        # Extract quaternion components
        x = msg.x
        y = msg.y
        z = msg.z
        w = msg.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = quaternion_to_euler(w, x, y, z)
        roll  = math.degrees(roll)* -1
        self.yaw_value.data = roll

        if self.yaw_value.data < 0:
            self.yaw_value.data += 360
            
        self.yaw_value.data += 180.76
        self.yaw_value.data = (self.yaw_value.data % 360) 
        self.yaw.publish(self.yaw_value)
        print(self.yaw_value.data)

        # Log the Euler angles
        # self.get_logger().info(

        #     f'Converted to Euler Angles -> Yaw: {self.yaw_value.data:.9f}'
        # )

def main(args=None):
    rclpy.init(args=args)
    node = QuaternionToEulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
