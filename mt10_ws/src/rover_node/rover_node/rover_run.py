"""
Bracu Mongol Tori(10) Control Base Code! Written by Marzanul Momenine With Love!
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from mt_arm_interfaces.msg import ControlMsgs
import rover_node.ardurover_utility as lib

class MT10(Node):
    def __init__(self, vehicle):
        super().__init__("Rover_Control")

        # Initialize vehicle
        if vehicle is None:
            self.get_logger().error("No Vehicle Found!")
            self.get_logger().error("Exiting!")
            rclpy.shutdown()
        else:
            self.get_logger().info("Vehicle Found!")
            self.get_logger().info("Initializing the Vehicle")
            self.rover = vehicle
            self.rover.init_connection()
            self.get_logger().info("\033[92mConnected to the vehicle!\033[0m")

        
        # IMU Data
        self.AHRS2 = None

        #LED Light
        self.rover.set_relay('blue')

        # Control Variables
        self.enable_control = False
        self.teleop_flag = False
        self.robotic_arm_flag = False

        # Scale Factors
        self.MAX_SCALE = 100
        self.MIN_SCALE = -100
        self.MAX_SCALE_D = 100
        self.MIN_SCALE_D = -100

        # Neutral Values
        self.NEUTRAL = 1500

        # Movement
        self.forward = 0
        self.turn = 0

        # Robotic Arm Control
        self.base = 0
        self.link1 = 0
        self.link2 = 0
        self.d1 = 0
        self.d2 = 0
        self.claw = 0

        # Subscribe to topics
        self.teleop = self.create_subscription(Twist, "/cmd_vel", self.teleop_callback, 1)
        self.robotic_arm = self.create_subscription(ControlMsgs, "/robotic_arm", self.robotic_arm_callback, 1)
        self.to_arm = self.create_subscription(Bool, "/arm_disarm", self.arm_disarm_callback, 1)
        self.light  = self.create_subscription(String, "/light_status", self.light_callback, 1)
        # self.mode = self.create_subscription(String, "/mode", self.mode_callback, 1)

        # Publish to topics
        # self.imu_pub = self.create_publisher(Imu, "/internal_ahrs2", 1)
        # self.gps_pub = self.create_publisher(NavSatFix, "/gps/coordinate", 0)
        # self.gps_rtk_pub = self.create_publisher(NavSatFix, "/gps_rtk", 0)

        # Timers
        self.control_timer = self.create_timer(1/10, self.control_loop)
        # self.internal_IMU = self.create_timer(1/20, self.imu_callback)
        # self.rtk_gps = self.create_timer(1, self.gps_callback)
    
    def light_callback(self, msg:String):
        self.rover.set_relay( rgb= msg.data)
        self.get_logger().info(f"Light Status: {msg.data}")

    def arm_disarm_callback(self, msg: Bool):
        """Callback to arm or disarm the vehicle."""
        self.enable_control = msg.data
        if msg.data:
            self.get_logger().info("Arming Mongol_tori")
            self.rover.arm()
            self.get_logger().info("\033[92mArmed!\033[0m")
        else:
            self.get_logger().error("Disarming Mongol_tori")
            self.rover.disarm()
            self.get_logger().error("Disarmed!")
            self.enable_control = False

    def teleop_callback(self, msg: Twist):
        """Callback to handle teleoperation commands."""
        self.raw_move = msg.linear.x
        self.raw_turn = msg.angular.z
        # print(self.raw_move, self.raw_turn)


        if self.raw_move == 0.0 and self.raw_turn == 0.0:
            self.teleop_flag = False
            self.forward = self.NEUTRAL
            self.turn = self.NEUTRAL
        elif self.raw_move == 0.0 and self.raw_turn != 0.0:
            self.teleop_flag = True
            self.forward = self.NEUTRAL
            self.turn = int(self.map_value(self.raw_turn, self.MIN_SCALE, self.MAX_SCALE, 1900, 1100))
        elif self.raw_move != 0.0 and self.raw_turn == 0.0:
            self.teleop_flag = True 
            self.forward = int(self.map_value(self.raw_move, self.MIN_SCALE, self.MAX_SCALE, 1900, 1100))
            self.turn = self.NEUTRAL
        else:
            self.teleop_flag = True
            self.forward = int(self.map_value(self.raw_move, self.MIN_SCALE, self.MAX_SCALE, 1900, 1100))
            self.turn = int(self.map_value(self.raw_turn, self.MIN_SCALE, self.MAX_SCALE, 1100, 1900))

    def robotic_arm_callback(self, msg: ControlMsgs):
        """Callback to handle robotic arm commands."""
        self.raw_base = msg.base
        self.raw_link1 = msg.link1
        self.raw_link2 = msg.link2
        self.raw_d1 = msg.d1
        self.raw_d2 = msg.d2
        self.raw_claw = msg.claw

        if all(v == 0.0 for v in [self.raw_base, self.raw_link1, self.raw_link2, self.raw_d1, self.raw_d2, self.raw_claw]):
            self.robotic_arm_flag = False
            self.base = self.NEUTRAL
            self.link1 = self.NEUTRAL
            self.link2 = self.NEUTRAL
            self.d1 = self.NEUTRAL
            self.d2 = self.NEUTRAL
            self.claw = self.NEUTRAL
        else:
            self.robotic_arm_flag = True
            self.base = int(self.map_value(self.raw_base, self.MIN_SCALE, self.MAX_SCALE, 800, 2200))
            self.link1 = int(self.map_value(self.raw_link1, self.MIN_SCALE, self.MAX_SCALE, 800, 2200))
            self.link2 = int(self.map_value(self.raw_link2, self.MIN_SCALE, self.MAX_SCALE, 2200, 800))
            self.d1 = int(self.map_value(self.raw_d1, self.MIN_SCALE_D, self.MAX_SCALE_D, 1600, 1300))
            self.d2 = int(self.map_value(self.raw_d2, self.MIN_SCALE_D, self.MAX_SCALE_D, 1350, 1650))
            self.claw = int(self.map_value(self.raw_claw, self.MIN_SCALE, self.MAX_SCALE, 1300, 1700))

    def control_loop(self):
        """Main control loop to send commands to the rover."""
        if self.enable_control and (self.teleop_flag or self.robotic_arm_flag):
            self.rover.set_rc_channel_pwm(
                pitch=int(self.forward),
                throttle=int(self.turn),
                base=int(self.base),
                link1=int(self.link1),
                link2=int(self.link2),
                d1=int(self.d1),
                d2=int(self.d2),
                claw=int(self.claw)
            )
            self.get_logger().info(f"{self.forward} || {self.turn} || {self.base} || {self.link1} || {self.link2} || {self.d1} || {self.d2} || {self.claw}")
            self.teleop_flag = False
            self.robotic_arm_flag = False
        elif self.enable_control:
            self.rover.set_rc_channel_pwm()
            self.get_logger().info("No Command Received")

    def map_value(self, x, in_min, in_max, out_min, out_max):
        """Helper function to map a value from one range to another."""
        if x == 0:
            return 1500
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def gps_callback(self):
        """Callback to publish GPS data."""
        
        gps_send = NavSatFix()
        if self.rover.get_gps() != None:
            self.GPS_RAW = self.rover.get_gps()
            gps_send.latitude = float("{:.10f}".format(self.GPS_RAW['lat'] * 0.0000001))
            gps_send.longitude = float("{:.10f}".format(self.GPS_RAW['lon'] * 0.0000001))
            self.accuracy = float(self.GPS_RAW['eph'])*0.01
            self.satellite = int(self.GPS_RAW['satellites_visible'])
            print(self.accuracy)
            self.get_logger().info(f"Lat: {float(gps_send.latitude)} Lon: {float(gps_send.longitude)}")
            self.gps_pub.publish(gps_send)

    def imu_callback(self):
        """Callback to publish IMU data."""
        self.AHRS2 = self.rover.get_ahrs2()
        msg = Imu()
        msg.header.frame_id = "IMU"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation.x = self.AHRS2["roll"]
        msg.orientation.y = self.AHRS2["pitch"]
        msg.orientation.z = self.AHRS2["yaw"]
        # print(msg)
        self.imu_pub.publish(msg)

    def mode_callback(self, msg: String):
        """Callback to handle mode changes."""
        current_mode = self.rover.get_current_mode()
        if msg.data == "MANUAL":
            if current_mode == "MANUAL":
                self.get_logger().info("Already in Manual Mode")
            self.rover.set_manual_mode()
            self.get_logger().info("Manual Mode")
        elif msg.data == "AUTO":
            if current_mode == "AUTO":
                self.get_logger().info("Already in Auto Mode")
            self.rover.set_auto_mode()
            self.get_logger().info("Auto Mode")
        elif msg.data == "SIMPLE":
            if current_mode == "SIMPLE":
                self.get_logger().info("Already in Simple Mode")
            self.rover.set_simple_mode()
            self.get_logger().info("Simple Mode")

    def delete(self):
        """Clean up resources and disarm the vehicle."""
        self.rover.disarm()
        self.rover.close_con()
        self.get_logger().fatal("Disconnected from the Vehicle")


def main():
    rclpy.init()
    global mongol_tori
    mongol_tori = MT10(lib.Vehicle("/dev/ttyACM0", 57600))
    rclpy.spin(mongol_tori)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Keyboard Interrupted")
        try:
            global mongol_tori
            mongol_tori.delete()
            mongol_tori.destroy_node()
            rclpy.shutdown()
        except:
            print("Unable to Disarm!")
