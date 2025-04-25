import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import cv_bridge

class ImageBlender(Node):
    def __init__(self):
        super().__init__("image_blender")
        # Subscribe to two image topics
        self.subscription1 = self.create_subscription(
            Image, "/zed_camera_35084019/left", self.image1_callback, 30)
        self.subscription2 = self.create_subscription(
            Image, "/zed_camera_32957407/right", self.image2_callback, 30)
        
        self.stitched_pub = self.create_publisher(
            Image,  # You can choose to publish as compressed or convert to Image
            "/stitched_image",
            10
        )

        self.bridge = cv_bridge.CvBridge()
        self.image1 = None
        self.image2 = None

        # Create a window and trackbars for adjusting alpha and gamma values.
        window_name = "Blended Image"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        # Create trackbar "alpha": range 0 to 100 (will be divided by 100 to yield 0.0 to 1.0)
        cv2.createTrackbar("alpha", window_name, 58, 100, lambda x: None)  # default at 50 → 0.5
        # Create trackbar "gamma": range 0 to 255
        cv2.createTrackbar("gamma", window_name, 0, 255, lambda x: None)

        # Timer callback to update the blended display (approximately 30Hz)
        self.timer = self.create_timer(0.067, self.timer_callback)

    def image1_callback(self, msg: Image):
        try:
            self.image1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image1: {e}")

    def image2_callback(self, msg: Image):
        try:
            self.image2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image2: {e}")

    def blend_images_with_trackbars(self, img1, img2, overlap=200):
        """
        Blends two images side by side with a specified overlap.
        The overlapping region is blended using the current trackbar values for alpha and gamma.
        """
        # Ensure both images have the same height. If not, resize img2.
        h1, w1, _ = img1.shape
        h2, w2, _ = img2.shape
        if h1 != h2:
            img2 = cv2.resize(img2, (w2, h1))
            h2, w2, _ = img2.shape

        # Ensure that both images are wide enough for the overlap.
        if w1 < overlap or w2 < overlap:
            self.get_logger().warn("One of the images is too narrow for the specified overlap.")
            return None

        # Divide each image into non-overlapping and overlapping parts.
        left_non_overlap = img1[:, :w1 - overlap]
        overlap_left = img1[:, w1 - overlap:]
        overlap_right = img2[:, :overlap]
        right_non_overlap = img2[:, overlap:]

        # Retrieve current trackbar positions.
        # Alpha trackbar: value between 0 and 100, divided by 100 to yield 0.0 to 1.0.
        alpha_trackbar = cv2.getTrackbarPos("alpha", "Blended Image") / 100.0
        beta = 1 - alpha_trackbar
        gamma = cv2.getTrackbarPos("gamma", "Blended Image")

        # Blend the overlapping region using cv2.addWeighted.
        blended_overlap = cv2.addWeighted(overlap_left, alpha_trackbar,
                                          overlap_right, beta, gamma)

        # Concatenate the non-overlapping parts and the blended overlap.
        blended_image = np.concatenate([left_non_overlap, blended_overlap, right_non_overlap], axis=1)
        return blended_image

    def timer_callback(self):
        if self.image1 is not None and self.image2 is not None:
            blended = self.blend_images_with_trackbars(self.image1, self.image2, overlap=200)

            ros_msg = self.bridge.cv2_to_imgmsg(blended, 'bgr8')
            ros_msg.header.stamp = self.get_clock().now().to_msg()
            self.stitched_pub.publish(ros_msg)
            if blended is not None:
                # cv2.imshow("Blended Image", blended)
                cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageBlender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
