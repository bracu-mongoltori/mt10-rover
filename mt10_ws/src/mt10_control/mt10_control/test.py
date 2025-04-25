import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sbg_driver.msg import SbgGpsPos
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class GNSSPublisher(Node):
    def __init__(self):
        super().__init__("gnss_publisher")
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,history=QoSHistoryPolicy.KEEP_ALL,depth=10)
        #self.gnss_publisher = self.create_publisher(SbgGpsPos, "/coordinates", 10,qos_profile)
        self.gnss_publisher = self.create_publisher(SbgGpsPos, "/coord_pub", qos_profile=qos_profile)
        self.target_publisher = self.create_publisher(String, "/target_name", qos_profile=qos_profile)
        self.gnss_result = self.create_subscription(
            String, "/autonomous_status", self.reached_callback, 10
        )
        self.gnss_points = []
        

    def load_gnss_points(self):
        with open(
            "/home/mt10/mt10_ws/src/mt10_control/mt10_control/gnss_points.txt", "r"
        ) as file:
            next(file)
            for line in file:
                target, lat, lon = line.strip().split(",")
                self.gnss_points.append((target, float(lat), float(lon)))

    def publish_next_point(self):
        if len(self.gnss_points) > 0:
            targ, lat, lon = self.gnss_points.pop(0)
            point = SbgGpsPos()
            point.latitude = lat
            point.longitude = lon

            target = String()
            target.data = targ

            self.gnss_publisher.publish(point)
            self.target_publisher.publish(target)
            self.get_logger().info(f"Published GNSS point: {lat}, {lon}")
            self.get_logger().info(f"Published target: {targ}")
        else:
            self.get_logger().info("All Points have been published")

    def reached_callback(self, msg):
        if msg.data == "reached":
            self.get_logger().info(msg.data)
            self.get_logger().info(f"Received reached message: {msg.data}")
            self.publish_next_point()


def main(args=None):
    rclpy.init(args=args)
    gnss_publisher = GNSSPublisher()
    
    gnss_publisher.load_gnss_points()
    gnss_publisher.publish_next_point()
    rclpy.spin(gnss_publisher)

    gnss_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
