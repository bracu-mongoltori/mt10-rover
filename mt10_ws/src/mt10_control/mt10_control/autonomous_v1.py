import rclpy
from rclpy.node import Node 
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sbg_driver.msg import SbgEkfEuler
from sbg_driver.msg import SbgGpsPos
from sbg_driver.msg import SbgEkfNav
from time import sleep
from math import radians, degrees, sin, cos, asin, sqrt, atan2


class Autonomous(Node):
    def __init__(self, max_lin_vel=60.0, max_ang_vel=100.0, yaw_threshold=15, distance_threshold=1.5):
        super().__init__("autonomous")
        self.autonomous = self.create_subscription(SbgGpsPos, "/coord_pub", self.coordinates_callback, 10)
        self.autonomous = self.create_subscription(String, "/target_name", self.coordinates_name_callback, 10)
        self.rover = self.create_publisher(Twist, "/cmd_vel", 10)
        self.gps = self.create_subscription(SbgGpsPos, "/sbg/gps_pos", self.gps_callback, 10)
        # self.gps = self.create_subscription(SbgEkfNav, "/sbg/ekf_nav", self.gps_callback, 10)
        self.orientation = self.create_subscription(SbgEkfEuler, "/sbg/ekf_euler", self.orientation_callback, 10)
        self.status_pub = self.create_publisher(String, "/status", 10)
        self.ar_resume = self.create_publisher(String, "/continue_search", 10)
        self.mallet_resume = self.create_publisher(String, "/continue_search", 10)
        self.bottle_resume = self.create_publisher(String, "/continue_search", 10)
        self.autonomous_status = self.create_publisher(String, "/autonomous_status", 10)
        self.arm_disarm_pub = self.create_publisher(Bool, "/arm_disarm", 10)
        self.status_timer = self.create_timer(0.2, self.status_stuff)
        self.autonomous_timer = self.create_timer(0.05, self.autonomous_callback)
        self.light_status_pub = self.create_publisher(String, "/light_status", 10)
        #self.log_timer = self.create_timer(0.2, self.log_callback)


        self.autonomous_on = False

        self.my_lat = 0.0
        self.my_lon = 0.0
        self.my_yaw = 0.0

        self.target_lat = 0.0
        self.target_lon = 0.0
        
        self.target_name = ""

        self.yaw_threshold = yaw_threshold
        self.distance_threshold = distance_threshold

        self.forward = Twist()
        self.left = Twist()
        self.right = Twist()
        self.backward = Twist()
        self.stop = Twist()

        self.forward.linear.x = max_lin_vel
        self.left.angular.z = max_ang_vel
        self.right.angular.z = -max_ang_vel
        self.backward.linear.x = -max_lin_vel
        self.stop.linear.x = 0.0
        self.stop.linear.z = 0.0

        self.prev_msg = None

        self.get_logger().info("Node initialized. Begin.")
        bool_msg = Bool()
        bool_msg.data = True
        self.arm_disarm_pub.publish(bool_msg)

    def distance_from_gps(self, lat1, lat2, lon1, lon2):
        #radius of earth in kilometers
        r = 6371

        lon1 = radians(lon1)
        lon2 = radians(lon2)
        lat1 = radians(lat1)
        lat2 = radians(lat2)
        
        #Haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * asin(sqrt(a))

        return c * r * 1000

    def bearing(self, curr_lat, curr_lon, target_lat, target_lon): #Bearing to waypoint (degrees)
        target_lat, target_lon, curr_lat, curr_lon = map(radians, [target_lat, target_lon, curr_lat, curr_lon])
        d_lon = target_lon - curr_lon
        return degrees(atan2(sin(d_lon) * cos(target_lat), cos(curr_lat) * sin(target_lat) - (sin(curr_lat) * cos(target_lat) * cos(d_lon))))%360

    def gps_callback(self, msg: SbgGpsPos):
    #def gps_callback(self, msg: SbgEkfNav):
        self.my_lat = msg.latitude
        self.my_lon = msg.longitude

    def orientation_callback(self, msg: SbgEkfEuler):
        self.my_yaw = degrees(msg.angle.z)%360
        # print(self.my_yaw)

    def status_stuff(self, msg=None):
        pubbed = String()
        if self.autonomous_on == False:
            msg = "Rover autonomous navigation not active"
        pubbed.data = str(msg)
        self.status_pub.publish(pubbed)
        if msg != None:
            self.get_logger().info(msg)
    
    def coordinates_callback(self, msg: SbgGpsPos):
    # def coordinates_callback(self, msg: SbgEkfNav):
        self.autonomous_on = True
        self.target_lat = msg.latitude
        self.target_lon = msg.longitude
        self.get_logger().info("Beginning Navigation")
        self.get_logger().info(f"Current distance to target {self.distance_from_gps(self.my_lat, self.target_lat, self.my_lon, self.target_lon)}")
        
    def coordinates_name_callback(self, msg: String):
        self.get_logger().info(f"Target Name: {msg.data}")
        self.target_name = msg.data
        
        

    def autonomous_callback(self):
        if self.autonomous_on == False:
            pass
        else:
            if self.distance_from_gps(self.my_lat, self.target_lat, self.my_lon, self.target_lon) < self.distance_threshold:
                msg = String()
                msg.data = "reached"
                print("Reached position")

                self.light_status_pub.publish(String(data="green"))

                if self.target_name == "4_point_travel":
                    self.ar_resume.publish(String(data="continue"))
                if self.target_name == "AR":
                    self.ar_resume.publish(String(data="continue"))
                if self.target_name == "GNSS":
                    self.autonomous_status.publish(msg)
                if self.target_name == "MALLET":
                    self.mallet_resume.publish(String(data="continue"))
                if self.target_name == "BOTTLE":
                    self.bottle_resume.publish(String(data="continue"))
                self.autonomous_on = False
                self.rover.publish(self.stop)
            else:
               #  self.rover.publish(self.stop) # Needed for Real Rover

                # distance = haversine_distance(lat, lon, target_lat, target_lon)
                target_yaw = self.bearing(self.my_lat, self.my_lon, self.target_lat, self.target_lon)

                # upper_limit = target_yaw + self.yaw_threshold
                # lower_limit = target_yaw - self.yaw_threshold

                d_yaw = abs(target_yaw - self.my_yaw)
                self.get_logger().info(f"yaw: {d_yaw}")

                if d_yaw > 15:
                    #if lower_limit < -135 and self.my_yaw > 90:
                     #   msg = self.left
                      #  turn_log = "turning left hard"        
                    #elif upper_limit > 135 and self.my_yaw < -90:
                     #   msg = self.right
                      #  turn_log = "turning right hard"

                    if self.my_yaw > target_yaw:
                        msg = self.left
                        turn_log = "turning left"
                    elif self.my_yaw < target_yaw:
                        msg = self.right
                        turn_log = "turning right"
                    else:
                        msg = self.forward
                        turn_log = "forward"

                    # if self.my_yaw > target_yaw:
                    #     if d_yaw >= 180.0:
                    #         msg = self.left
                    #         turn_log = "turning left"

                    #     else:
                    #         msg = self.right
                    #         turn_log = "turning right"

                    # elif self.my_yaw < target_yaw:
                    #     if d_yaw >= 180.0:
                    #         msg = self.right
                    #         turn_log = "turning right"

                    #     else:
                    #         msg = self.left
                    #         turn_log = "turning left"
                    # else:
                    #     msg = self.forward
                    #     turn_log = "forward"



                else:
                    msg = self.forward
                    turn_log = "forward"

                self.rover.publish(msg)
                self.status_stuff("GPS: ")
                self.status_stuff(f"lat: {self.my_lat}, lon: {self.my_lon}")
                self.status_stuff(f"target lat: {self.target_lat}, target lon: {self.target_lon}")
                self.status_stuff(f"Current distance to target: {self.distance_from_gps(self.my_lat, self.target_lat, self.my_lon, self.target_lon)}")
                self.status_stuff(f"Current yaw: {self.my_yaw}, target yaw: {target_yaw}")
                self.status_stuff(f"Angle left: {abs(target_yaw - self.my_yaw)}")
                self.status_stuff("")
                self.status_stuff(f"{turn_log}")
                    
                        
                # sleep(0.2)

def main():
    rclpy.init()
    autonomous = Autonomous()

    rclpy.spin(autonomous)

    autonomous.destroy_node()
    rclpy.shutdowm()

if __name__ == "__main__":
    main()

        
