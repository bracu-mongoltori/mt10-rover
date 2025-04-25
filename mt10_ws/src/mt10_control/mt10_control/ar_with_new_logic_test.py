import cv2
import numpy as np
from cv2 import aruco
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64
from geometry_msgs.msg import Twist
# from sbg_driver.msg import SbgEkfEuler
import time
from math import degrees

# Constants from original code
MARKER_SIZE = 0.15
RECT_WIDTH = 150
RECT_HEIGHT = 90
LINEAR_SPEED = 80.0
ANGULAR_SPEED = 80.0
STOP_DISTANCE = 1.5
TRACKING_TIMEOUT = 1.5  # Tolerance time in seconds for losing ArUco while tracking

class ArucoSearchTrackNode(Node):
    def __init__(self):
        super().__init__('aruco_search_track_node')
        
        # Publishers
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detection_status_pub = self.create_publisher(String, '/aruco_detection_status', 10)
        self.detection_status_msg = String()
        
        # Subscribers
        # self.orientation = self.create_subscription(
        #     SbgEkfEuler, "/sbg/ekf_euler", self.orientation_callback, 10
        # )

        self.orientation = self.create_subscription(
            Float64, "/witmotion_eular/yaw", self.orientation_callback, 10
        )
        self.continue_sub = self.create_subscription(
            String, "/continue_search", self.continue_callback, 10
        )
        
        self.target_reached_pub = self.create_publisher(String, '/autonomous_status', 10)
        
        self.point_publish_cmd = self.create_publisher(String, '/point_publish_cmd', 10)
        
        # Initialize video capture
        self.cap = cv2.VideoCapture(0)
        
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        
        # Load ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.parameters.errorCorrectionRate = 0.9
        
        # Create timer for camera callback
        self.timer = self.create_timer(0.0167, self.camera_callback)  # 60Hz
        
        # State variables
        self.state = "WAITING"
        self.detection_start_time = time.time()
        self.last_tracking_time = None
        self.last_instruction = None
        self.current_angle = 0.0
        self.target_angle = 60.0
        self.total_rotation = 0.0
        self.my_yaw = 0.0
        self.start_yaw = None
        
        self.get_logger().info('ArUco Search and Track Node initialized')
        time.sleep(3)
        self.detection_status_msg.data = 'ArUco Search and Track Node initialized'
        self.detection_status_pub.publish(self.detection_status_msg)
    
    def continue_callback(self, msg: String):
        if self.state == "WAITING" and msg.data == "continue":
            self.get_logger().info('Received continue command, resuming search')
            self.detection_status_msg.data = 'Received continue command, resuming search'
            self.detection_status_pub.publish(self.detection_status_msg)
            self.state = "DETECTING"
            self.detection_start_time = time.time()
            self.total_rotation = 0.0
            self.start_yaw = None
            
    def publish_target_reached(self):
        msg = String()
        msg.data = 'reached'
        self.target_reached_pub.publish(msg)
        self.get_logger().info('Published target reached message')
        self.detection_status_msg.data = 'Published target reached message'
        self.detection_status_pub.publish(self.detection_status_msg)
    
    def orientation_callback(self, msg: Float64):
        self.my_yaw = msg.data
    
    # [Previous helper functions remain the same: calculate_distance, draw_guides, get_movement_instruction, ar_detection]
    
    def get_movement_instruction(self, marker_center, rect_bounds, distance): # get movement instruction based on marker position
        if distance < STOP_DISTANCE:
            return "Stop"
        rect_x1, _, rect_x2, _ = rect_bounds
        x, _ = marker_center
        
        if x < rect_x1:
            return "Move Left"
        elif x > rect_x2:
            return "Move Right"
        else:
            return "Move Forward"
        
    def ar_detection(self, frame):
        display_frame = frame.copy() # create a copy of the frame to draw on it
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # convert to grayscale
        
        corners, ids, _ = aruco.detectMarkers(display_frame, self.aruco_dict, parameters=self.parameters)
        rect_bounds = self.draw_guides(frame, display_frame)
        
        return corners, ids, display_frame, rect_bounds
    
    def calculate_distance(self, marker_center, image_center, marker_size_pixels):
        focal_length_pixels = 900 # Logitech good one
        distance = (MARKER_SIZE * focal_length_pixels) / marker_size_pixels
        return distance
    
    def draw_guides(self, frame, draw_frame):
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        
        cv2.line(draw_frame, (0, center_y), (width, center_y), (255, 255, 255), 1)
        cv2.line(draw_frame, (center_x, 0), (center_x, height), (255, 255, 255), 1)
        
        rect_x1 = center_x - RECT_WIDTH // 2
        rect_y1 = center_y - RECT_HEIGHT // 2
        rect_x2 = center_x + RECT_WIDTH // 2
        rect_y2 = center_y + RECT_HEIGHT // 2
        cv2.rectangle(draw_frame, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 255, 0), 1)
        
        return rect_x1, rect_y1, rect_x2, rect_y2
    
    def start_rotation(self):
        self.state = "ROTATING"
        if self.start_yaw is None:
            self.start_yaw = self.my_yaw
        msg = Twist()
        
        msg.angular.z = -ANGULAR_SPEED
        self.vel_publisher.publish(msg)
                
    def publish_movement_command(self, instruction):
        msg = Twist()
        
        if instruction == "Stop":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('Target reached! Stopping robot. Waiting for continue command...')
            self.detection_status_msg.data = 'Target reached! Stopping robot. Waiting for continue command...'
            self.detection_status_pub.publish(self.detection_status_msg)
            
            self.vel_publisher.publish(msg)
            self.last_instruction = instruction
            self.get_logger().info(f"Ins stop: {self.last_instruction}")
            self.detection_status_msg.data = f"Ins stop: {self.last_instruction}"
            self.detection_status_pub.publish(self.detection_status_msg)

            return True
        else:
            if instruction == "Move Forward":
                msg.linear.x = LINEAR_SPEED
            elif instruction == "Move Left":
                msg.angular.z = ANGULAR_SPEED
            elif instruction == "Move Right":
                msg.angular.z = -ANGULAR_SPEED
        
        self.last_instruction = instruction
        
        self.vel_publisher.publish(msg)
            
        # self.vel_publisher.publish(msg)
        self.get_logger().info(f"Ins all: {self.last_instruction}")
        self.detection_status_msg.data = f"Ins all: {self.last_instruction}"
        self.detection_status_pub.publish(self.detection_status_msg)
        return False
    
    def handle_tracking_loss(self):
        current_time = time.time()
        if self.last_tracking_time is None:
            self.last_tracking_time = current_time
            
        time_since_last_track = current_time - self.last_tracking_time
        
        if time_since_last_track < TRACKING_TIMEOUT:
            # Continue last movement whether it was forward or turning
            msg = Twist()
            if self.last_instruction == "Move Forward":
                self.get_logger().info('ArUco temporarily lost, continuing forward motion')
                self.detection_status_msg.data = 'ArUco temporarily lost, continuing forward motion'
                self.detection_status_pub.publish(self.detection_status_msg)
                msg.linear.x = LINEAR_SPEED
            elif self.last_instruction == "Move Left":
                self.get_logger().info('ArUco temporarily lost, continuing left turn')
                self.detection_status_msg.data = 'ArUco temporarily lost, continuing left turn'
                self.detection_status_pub.publish(self.detection_status_msg)
                msg.angular.z = ANGULAR_SPEED
            elif self.last_instruction == "Move Right":
                self.get_logger().info('ArUco temporarily lost, continuing right turn')
                self.detection_status_msg.data = 'ArUco temporarily lost, continuing right turn'
                self.detection_status_pub.publish(self.detection_status_msg)
                msg.angular.z = -ANGULAR_SPEED
            
            self.vel_publisher.publish(msg)
            return True
        else:
            # If we've exceeded the timeout, stop and switch to detection
            self.get_logger().info('ArUco lost for too long, switching to detection')
            self.detection_status_msg.data = 'ArUco lost for too long, switching to detection'
            self.detection_status_pub.publish(self.detection_status_msg)
            msg = Twist()
            self.vel_publisher.publish(msg)
            self.state = "DETECTING"
            self.detection_start_time = current_time
            self.last_tracking_time = None
            self.last_instruction = None
            return False
    
    def camera_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            self.detection_status_msg.data = 'Failed to capture frame'
            self.detection_status_pub.publish(self.detection_status_msg)
            return
        
        corners, ids, display_frame, rect_bounds = self.ar_detection(frame)
        current_time = time.time()
        
        if self.state == "WAITING":
            self.get_logger().info("Waiting...")
            self.detection_status_msg.data = "Waiting..."
            self.detection_status_pub.publish(self.detection_status_msg)
            # Just update display, no movement
            if ids is not None:
                aruco.drawDetectedMarkers(display_frame, corners)
        
        elif self.state == "DETECTING":
            if ids is not None:
                self.state = "TRACKING"
                self.last_tracking_time = current_time
                self.get_logger().info('ArUco detected, starting tracking')
                self.detection_status_msg.data = 'ArUco detected, starting tracking'
                self.detection_status_pub.publish(self.detection_status_msg)
            elif current_time - self.detection_start_time >= 3.0:
                self.start_rotation()
                self.detection_start_time = current_time
        
        elif self.state == "ROTATING":
            if self.start_yaw is not None:
                angle_diff = ((self.my_yaw - self.start_yaw)) % 360
                self.get_logger().info(f"Angle diff= {angle_diff}")
                self.detection_status_msg.data = f"Angle diff= {angle_diff}"
                self.detection_status_pub.publish(self.detection_status_msg)
                self.get_logger().info(f"Total rotation= {self.total_rotation}")
                self.detection_status_msg.data = f"Total rotation= {self.total_rotation}"
                self.detection_status_pub.publish(self.detection_status_msg)
                if angle_diff >= 60.0 and angle_diff<= 350.0:
                    self.total_rotation += 60.0
                    self.start_yaw = self.my_yaw
                    if self.total_rotation >= 360.0:
                        self.get_logger().info('360 rotation done, ArUco not found. Waiting for next decision.')
                        self.detection_status_msg.data = '360 rotation done, ArUco not found. Waiting for next decision.'
                        self.detection_status_pub.publish(self.detection_status_msg)
                        msg = Twist()
                        self.vel_publisher.publish(msg)
                        self.state = "WAITING"
                        self.point_publish_cmd.publish(String(data="go to point"))
                        return
                    
                    msg = Twist()
                    self.vel_publisher.publish(msg)
                    self.state = "DETECTING"
                    self.detection_start_time = current_time
                    self.start_yaw = None
                
        
        elif self.state == "TRACKING":
            if ids is not None:
                self.last_tracking_time = current_time
                for i in range(len(ids)):
                    marker_corners = corners[i][0]
                    marker_center = tuple(np.mean(marker_corners, axis=0).astype(int))
                    marker_width_pixels = np.linalg.norm(marker_corners[0] - marker_corners[1])
                    
                    distance = self.calculate_distance(marker_center, frame.shape[:2], marker_width_pixels)
                    instruction = self.get_movement_instruction(marker_center, rect_bounds, distance)
                    
                    self.get_logger().info(f"Distance to marker: {distance:.2f} m")
                    self.detection_status_msg.data = f"Distance to marker: {distance:.2f} m"
                    self.detection_status_pub.publish(self.detection_status_msg)
                    self.get_logger().info(f"Movement instruction: {instruction}")
                    self.detection_status_msg.data = f"Movement instruction: {instruction}"
                    self.detection_status_pub.publish(self.detection_status_msg)
                    
                    target_reached = self.publish_movement_command(instruction)
                    
                    aruco.drawDetectedMarkers(display_frame, corners)
                    
                    # Display information
                    cv2.putText(display_frame, f"ID: {ids[i][0]} Distance: {distance:.2f}m", 
                            (10, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    instruction_color = (0, 0, 255) if instruction == "Stop" else (255, 255, 255)
                    cv2.putText(display_frame, f"Instruction: {instruction}", 
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2)
                    
                    if target_reached:
                        self.state = "WAITING"
                        self.publish_target_reached()
            else:
                # Handle temporary loss of tracking
                continuing = self.handle_tracking_loss()
                if continuing:
                    cv2.putText(display_frame, "ArUco temporarily lost, continuing movement", 
                            (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # Display state
        cv2.putText(display_frame, f"State: {self.state}", 
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        
        cv2.imshow('Aruco Search and Track', display_frame)
        cv2.waitKey(1)
    
    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoSearchTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
