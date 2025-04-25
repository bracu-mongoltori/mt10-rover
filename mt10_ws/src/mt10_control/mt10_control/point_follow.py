import rclpy
from rclpy.node import Node
from sbg_driver.msg import SbgGpsPos
from std_msgs.msg import String
from math import radians, cos, sin, degrees

class GPSSquareNode(Node):
    def __init__(self):
        super().__init__('gps_square_node')
        self.subscription = self.create_subscription(
            SbgGpsPos,
            '/sbg/gps_pos',  # Replace with the actual topic name
            self.gps_callback,
            10
        )
        
        self.point_publish_cmd = self.create_subscription(String, '/point_publish_cmd', self.point_publish_cmd_callback, 10)
        self.point_publisher = self.create_publisher(SbgGpsPos, '/coord_pub', 10)
        self.point_name_pub = self.create_publisher(String, '/target_name', 10)
        self.square_points = []  # Array to store the square points
        self.side_length = 4.0  # Side length of the square in meters
        self.EARTH_RADIUS = 6371000.0  # Earth radius in meters
        self.lat = 0.0  # Current latitude
        self.lon = 0.0  # Current longitude
        self.vertices = []

    def gps_callback(self, msg: SbgGpsPos):

        # Current GPS location
        self.lat = msg.latitude
        self.lon = msg.longitude
        
    def point_publish_cmd_callback(self, msg: String):
        if msg.data == "go to point":
            if self.vertices == []:
                self.vertices = self.calculate_square_vertices(self.lat, self.lon)
            
            point = SbgGpsPos()
            point.latitude = self.vertices[0][0]
            point.longitude = self.vertices[0][1]
            self.point_publisher.publish(point)
            point_name = String()
            point_name.data = "4_point_travel"
            self.point_name_pub.publish(point_name)
            self.vertices.pop(0)
                


    def calculate_square_vertices(self, lat, lon):
        vertices = []
        half_side = self.side_length / 2.0

        # Angles for square corners (0, 90, 180, 270 degrees relative to center)
        angles = [0, 90, 180, 270]

        for angle in angles:
            angle_rad = radians(angle)
            delta_lat = (half_side / self.EARTH_RADIUS) * cos(angle_rad)
            delta_lon = (half_side / (self.EARTH_RADIUS * cos(radians(lat)))) * sin(angle_rad)

            vertex_lat = lat + degrees(delta_lat)
            vertex_lon = lon + degrees(delta_lon)
            vertices.append((vertex_lat, vertex_lon))

        return vertices

def main(args=None):
    rclpy.init(args=args)
    node = GPSSquareNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
