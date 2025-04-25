import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyzed.sl as sl
import threading
import cv2
import time
import math  # <-- Added import for math

class AutoZedPublisher(Node):
    def __init__(self):
        super().__init__('auto_zed_publisher')
        
        # Parameters
        self.declare_parameter('resolution', 'HD720')
        self.declare_parameter('fps', 60)
        self.declare_parameter('topic_prefix', 'zed_camera')

        # Automatically detect connected cameras
        self.cameras = []
        self.bridge = CvBridge()
        self.stop_signal = False

        # Subscribe to /center for the (x,y) coordinate string.
        self.cord_sub = self.create_subscription(String, "/center", self.depth_calc_callback, 10)  # <-- Added

        self.dist = self.create_publisher(String, "/distance", 10)
        
        # Get list of connected ZED cameras
        dev_list = sl.Camera.get_device_list()
        if len(dev_list) == 0:
            self.get_logger().error("No ZED cameras detected!")
            raise RuntimeError("No cameras found")

        # Initialize all detected cameras
        self.initialize_cameras(dev_list)

        self.get_logger().info(f"Found and initialized {len(self.cameras)} ZED cameras")

    def initialize_cameras(self, dev_list):
        """Initialize all detected ZED cameras"""
        topic_prefix = self.get_parameter('topic_prefix').value
        
        for idx, dev in enumerate(dev_list):
            try:
                # Create camera object
                zed = sl.Camera()
                
                # Set initialization parameters
                init_params = sl.InitParameters()
                init_params.set_from_serial_number(dev.serial_number)
                init_params.camera_resolution = self.get_resolution_enum()
                init_params.camera_fps = self.get_parameter('fps').value
                init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # Use NEURAL depth mode
                init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
                
                # Open camera
                status = zed.open(init_params)
                if status != sl.ERROR_CODE.SUCCESS:
                    self.get_logger().error(f"Failed to open camera {dev.serial_number}: {status}")
                    continue
                
                # Create publishers for left and right views
                topic_name_left = f"{topic_prefix}_{dev.serial_number}/left"
                topic_name_right = f"{topic_prefix}_{dev.serial_number}/right"
                publisher_left = self.create_publisher(Image, topic_name_left, 10)
                publisher_right = self.create_publisher(Image, topic_name_right, 10)
                
                # Store camera resources
                self.cameras.append({
                    'zed': zed,
                    'publisher_left': publisher_left,
                    'publisher_right': publisher_right,
                    'serial': dev.serial_number,
                    'thread': None
                })
                
                # Start capture thread for stereo (left & right)
                self.cameras[-1]['thread'] = threading.Thread(
                    target=self.camera_thread,
                    args=(zed, publisher_left, publisher_right, dev.serial_number)
                )
                self.cameras[-1]['thread'].start()
                
                self.get_logger().info(
                    f"Initialized camera {idx+1} (S/N: {dev.serial_number}) on topics '{topic_name_left}' and '{topic_name_right}'"
                )

            except Exception as e:
                self.get_logger().error(f"Error initializing camera {dev.serial_number}: {str(e)}")

    def get_resolution_enum(self):
        resolution_map = {
            'HD2K': sl.RESOLUTION.HD2K,
            'HD1080': sl.RESOLUTION.HD1080,
            'HD720': sl.RESOLUTION.HD720,
            'VGA': sl.RESOLUTION.VGA
        }
        return resolution_map[self.get_parameter('resolution').value]

    def camera_thread(self, camera, publisher_left, publisher_right, serial):
        """Thread function for continuous stereo image capture and publishing."""
        runtime = sl.RuntimeParameters()
        # Allocate two Mats: one for left view and one for right view.
        left_image = sl.Mat()
        right_image = sl.Mat()
        
        self.get_logger().info(f"Starting capture thread for camera {serial}")
        
        while not self.stop_signal:
            try:
                if camera.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                    # Retrieve left and right images
                    camera.retrieve_image(left_image, sl.VIEW.LEFT)
                    camera.retrieve_image(right_image, sl.VIEW.RIGHT)
                    
                    # Get the OpenCV images from the ZED Mat objects.
                    left_cv_image = left_image.get_data()
                    right_cv_image = right_image.get_data()

                    # Convert from RGBA to BGR (remove alpha channel)
                    left_cv_image = left_cv_image[...,:3]
                    right_cv_image = right_cv_image[...,:3]
                    
                    # Publish left and right images
                    publisher_left.publish(
                        self.bridge.cv2_to_imgmsg(left_cv_image, 'bgr8')
                    )
                    publisher_right.publish(
                        self.bridge.cv2_to_imgmsg(right_cv_image, 'bgr8')
                    )
            except Exception as e:
                self.get_logger().error(f"Camera {serial} error: {str(e)}")
                time.sleep(0.1)

    # <-- Added new callback for depth calculation based on /center message
    def depth_calc_callback(self, msg: String):
        try:
            coords = msg.data.split(',')
            if len(coords) != 2:
                self.get_logger().error("Invalid coordinate format. Expected 'x,y'")
                return
            x = int(coords[0].strip())
            y = int(coords[1].strip())
        except Exception as e:
            self.get_logger().error("Error parsing coordinate: " + str(e))
            return

        # Identify the two cameras based on serial number:
        first_camera = None  # Should be the one with serial ending "19"
        second_camera = None
        
        for cam in self.cameras:
            # Convert the serial to string if necessary.
            if str(cam['serial']).endswith("19"):
                first_camera = cam
            else:
                second_camera = cam

        if first_camera is None or second_camera is None:
            self.get_logger().error("Could not identify both cameras based on serial numbers.")
            return

        # Decide which camera to use based on the x coordinate:
        # Assume each camera produces an image of width 1280.
        if x >= 1280:
            # Use second camera and adjust x coordinate relative to its image.
            selected_cam = second_camera
            x = (x+200) - 1280
            self.get_logger().info("Using second camera for depth calculation")
        else:
            # Use first camera.
            selected_cam = first_camera
            self.get_logger().info("Using first camera for depth calculation")
            
            
        zed = selected_cam['zed']
        runtime_parameters = sl.RuntimeParameters()
        depth = sl.Mat()
        point_cloud = sl.Mat()


        # Attempt to grab a frame from the selected camera.
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            
            err, point_cloud_value = point_cloud.get_value(x, y)
            if math.isfinite(point_cloud_value[2]):
                distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                    point_cloud_value[1] * point_cloud_value[1] +
                                    point_cloud_value[2] * point_cloud_value[2])
                self.get_logger().info(f"Distance at pixel ({x},{y}): {distance:.2f} mm")
                distance_msg = String()
                distance_msg.data = f"{distance:.2f}"
                self.dist.publish(distance_msg)
            else:
                self.get_logger().info(f"Cannot compute distance at pixel ({x},{y})")
                distance_msg = String()
                distance_msg.data = "NaN"
                self.dist.publish(distance_msg)
        else:
            self.get_logger().error("Failed to grab image from camera for depth measurement")
    # <-- End of added callback

    def __del__(self):
        """Cleanup resources"""
        self.stop_signal = True
        for cam in self.cameras:
            if cam['thread'] is not None:
                cam['thread'].join()
            cam['zed'].close()
        self.get_logger().info("All cameras closed properly")

def main(args=None):
    rclpy.init(args=args)
    node = AutoZedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
