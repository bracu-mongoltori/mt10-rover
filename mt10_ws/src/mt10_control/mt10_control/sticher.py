#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Use CompressedImage type
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

class ImageStitcher(Node):
    def __init__(self):
        super().__init__('image_stitcher')
        
        # Parameters (ensure these topics match your published message types)
        self.declare_parameter('camera1_topic', '/zed_camera_35084019/left')
        self.declare_parameter('camera2_topic', '/zed_camera_32957407/left')
        self.declare_parameter('stitched_topic', '/stitched_image')
        self.declare_parameter('target_height', 1080)

        # Setup subscribers using CompressedImage type
        self.bridge = CvBridge()
        self.cam1_sub = Subscriber(self, Image, self.get_parameter('camera1_topic').value)
        self.cam2_sub = Subscriber(self, Image, self.get_parameter('camera2_topic').value)

        # Setup synchronizer with a forgiving slop (time tolerance)
        queue_size = 10
        self.ts = ApproximateTimeSynchronizer(
            [self.cam1_sub, self.cam2_sub],
            queue_size,
            0.1  # Increased tolerance
        )
        self.ts.registerCallback(self.stitch_callback)

        # Publisher (publishing uncompressed images; adjust if needed)
        self.stitched_pub = self.create_publisher(
            Image,  # You can choose to publish as compressed or convert to Image
            self.get_parameter('stitched_topic').value,
            10
        )

    def stitch_callback(self, msg1, msg2):
        cv_img1 = self.convert_image(msg1)
        cv_img2 = self.convert_image(msg2)

        cv_img1 = self.zoom_frame(cv_img1)

        # Check if any image is empty
        if cv_img1 is None or cv_img2 is None:
            self.get_logger().error("One of the images is empty. Skipping stitching.")
            return
        
        # Log image shapes for debugging
        self.get_logger().info(f"Image 1 shape: {cv_img1.shape}")
        self.get_logger().info(f"Image 2 shape: {cv_img2.shape}")

        # cv2.imshow("img1", cv_img1)


        # Use OpenCV's stitcher
        stitcher = cv2.Stitcher_create()
        # stitcher = cv2.Stitcher_create()

        status, stitched = stitcher.stitch([cv_img1, cv_img2])

        # stitched = cv2.hconcat([cv_img1,cv_img2])
        # if status != cv2.Stitcher_OK:
        #     self.get_logger().error(f"Stitching failed with status code {status}")
        #     return

        # cv2.imshow("stitched", stitched)
        # cv2.waitKey(1)

        # Convert the stitched image back to a ROS Image message
        ros_msg = self.bridge.cv2_to_imgmsg(stitched, 'bgr8')
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        self.stitched_pub.publish(ros_msg)

    def convert_image(self, msg):
        # Convert the incoming compressed data into a numpy array.
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # cv2.imshow("img", cv_image)

        return cv_image
        

    
    def resize_image(self, image, target_height):
        h, w = image.shape[:2]
        scale = target_height / h
        return cv2.resize(image, (int(w * scale), target_height))
    
    def zoom_frame(self, img, zoom_factor=1.15):
        h, w = img.shape[:2]
        new_w, new_h = int(w * zoom_factor), int(h * zoom_factor)

        # Resize the image
        zoomed_img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # Crop center to keep original size
        x1, y1 = (new_w - w) // 2, (new_h - h) // 2
        cropped_img = zoomed_img[y1:y1 + h, x1:x1 + w]

        return cropped_img


def main(args=None):
    rclpy.init(args=args)
    stitcher = ImageStitcher()
    rclpy.spin(stitcher)
    stitcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
