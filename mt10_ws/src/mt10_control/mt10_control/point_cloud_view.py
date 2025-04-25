import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import laspy
import numpy as np

class LazPublisher(Node):
    def __init__(self):
        super().__init__('laz_publisher')
        self.publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)
        self.las = laspy.read('/home/mt/new_point_cloud')  # Update path
        self.points = np.vstack([self.las.x, self.las.y, self.las.z]).T.astype(np.float32)
        self.timer = self.create_timer(1.0, self.publish_cloud)
        
    def publish_cloud(self):
        header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        pc2_msg = point_cloud2.create_cloud_xyz32(header, self.points)
        self.publisher.publish(pc2_msg)
        self.get_logger().info('Publishing PointCloud2')

def main(args=None):
    rclpy.init(args=args)
    node = LazPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
