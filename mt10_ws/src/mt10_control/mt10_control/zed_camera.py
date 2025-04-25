import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyzed.sl as sl
import threading
import cv2
import time
import math

class AutoZedPublisher(Node):
    def __init__(self):
        super().__init__('auto_zed_publisher')
        
        # Parameters
        self.declare_parameter('resolution', 'HD720')
        self.declare_parameter('fps', 60)
        self.declare_parameter('topic_prefix', 'zed_camera')

        # Camera resources
        self.camera = None
        self.bridge = CvBridge()
        self.stop_signal = False

        # Subscribe to /center for coordinates
        self.cord_sub = self.create_subscription(String, "/center", self.depth_calc_callback, 10)
        self.dist = self.create_publisher(String, "/distance", 10)
        
        # Get connected camera
        dev_list = sl.Camera.get_device_list()
        if len(dev_list) == 0:
            self.get_logger().error("No ZED cameras detected!")
            raise RuntimeError("No cameras found")

        # Initialize first found camera
        self.initialize_camera(dev_list[0])
        self.get_logger().info(f"Initialized ZED camera (S/N: {dev_list[0].serial_number})")

    def initialize_camera(self, dev):
        """Initialize a single ZED camera"""
        try:
            # Create and configure camera
            zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.set_from_serial_number(dev.serial_number)
            init_params.camera_resolution = self.get_resolution_enum()
            init_params.camera_fps = self.get_parameter('fps').value
            init_params.depth_mode = sl.DEPTH_MODE.NEURAL
            init_params.coordinate_units = sl.UNIT.METER

            # Open camera
            status = zed.open(init_params)
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"Failed to open camera: {status}")

            # Create publishers
            topic_prefix = self.get_parameter('topic_prefix').value
            self.publisher_left = self.create_publisher(Image, f"{topic_prefix}/left", 10)
            self.publisher_right = self.create_publisher(Image, f"{topic_prefix}/right", 10)

            # Start capture thread
            self.camera = zed
            self.camera_thread = threading.Thread(
                target=self.camera_loop,
                args=(zed,)
            )
            self.camera_thread.start()

        except Exception as e:
            self.get_logger().error(f"Camera initialization failed: {str(e)}")
            raise

    def get_resolution_enum(self):
        resolution_map = {
            'HD2K': sl.RESOLUTION.HD2K,
            'HD1080': sl.RESOLUTION.HD1080,
            'HD720': sl.RESOLUTION.HD720,
            'VGA': sl.RESOLUTION.VGA
        }
        return resolution_map[self.get_parameter('resolution').value]

    def camera_loop(self, camera):
        """Thread function for continuous image capture"""
        runtime = sl.RuntimeParameters()
        left_image = sl.Mat()
        right_image = sl.Mat()
        
        self.get_logger().info("Starting camera capture thread")
        
        while not self.stop_signal and rclpy.ok():
            try:
                if camera.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                    # Retrieve and publish images
                    camera.retrieve_image(left_image, sl.VIEW.LEFT)
                    camera.retrieve_image(right_image, sl.VIEW.RIGHT)
                    
                    left_cv = left_image.get_data()[..., :3]  # Remove alpha channel
                    right_cv = right_image.get_data()[..., :3]

                    self.publisher_left.publish(
                        self.bridge.cv2_to_imgmsg(left_cv, 'bgr8')
                    )
                    self.publisher_right.publish(
                        self.bridge.cv2_to_imgmsg(right_cv, 'bgr8')
                    )
            except Exception as e:
                self.get_logger().error(f"Camera error: {str(e)}")
                time.sleep(0.1)

    def depth_calc_callback(self, msg: String):
        """Handle depth calculation for received coordinates"""
        try:
            x, y = map(int, msg.data.split(','))
            if not self.camera:
                raise RuntimeError("Camera not initialized")
        except Exception as e:
            self.get_logger().error(f"Invalid input: {str(e)}")
            return

        # Depth measurement setup
        runtime = sl.RuntimeParameters()
        depth = sl.Mat()
        point_cloud = sl.Mat()


        # Attempt to grab a frame from the selected camera.
        if self.camera.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            self.camera.retrieve_measure(depth, sl.MEASURE.DEPTH)
            self.camera.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            
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

    def __del__(self):
        """Cleanup resources"""
        self.stop_signal = True
        if self.camera_thread is not None:
            self.camera_thread.join()
        if self.camera is not None:
            self.camera.close()
        self.get_logger().info("Camera resources released")

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