#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from sbg_driver.msg import SbgGpsPos, SbgEkfEuler


class SbgEllipsePublisher(Node):
    def __init__(self):
        super().__init__("sbg_ellipse_publisher")
        self.euler_publisher_ = self.create_publisher(SbgEkfEuler, "/sbg/ekf_euler", 10)
        self.gps_publisher_ = self.create_publisher(SbgGpsPos, "/sbg/gps_pos", 10)

    def publish_euler_message(self):
        # SbgEkfEuler message (for angles and accuracy)
        ekf_msg = SbgEkfEuler()

        # Header
        ekf_msg.header = Header()
        ekf_msg.header.stamp = self.get_clock().now().to_msg()
        ekf_msg.header.frame_id = "sbg_ellipse_frame"

        # Time since sensor is powered up (example timestamp in microseconds)
        ekf_msg.time_stamp = 123456

        # Roll, Pitch, Yaw (angles in radians)
        ekf_msg.angle = Vector3()
        ekf_msg.angle.x = float(
            input("Enter roll/msg.x (radians): ")
        )  # Roll in radians
        ekf_msg.angle.y = float(
            input("Enter pitch/msg.y (radians): ")
        )  # Pitch in radians
        ekf_msg.angle.z = float(
            input("Enter yaw/msg.z (radians): ")
        )  # Yaw (heading) in radians

        # Roll, Pitch, Yaw accuracy (1 sigma, in radians)
        ekf_msg.accuracy = Vector3()
        ekf_msg.accuracy.x = 0.01  # Accuracy of Roll
        ekf_msg.accuracy.y = 0.01  # Accuracy of Pitch
        ekf_msg.accuracy.z = 0.05  # Accuracy of Yaw

        # Publish the SbgEkfEuler message
        self.euler_publisher_.publish(ekf_msg)
        # Example logging
        self.get_logger().info(f"Publishing SbgEkfEuler: {ekf_msg}")

    def publish_gps_message(self):
        # SbgGpsPos message (for GPS-related data, if needed)
        gps_msg = SbgGpsPos()

        # Fill GPS data here (dummy values for now)
        gps_msg.header = Header()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "sbg_ellipse_frame"

        gps_msg.latitude = float(input("Enter latitude: "))
        gps_msg.longitude = float(input("Enter longitude: "))
        gps_msg.altitude = 100.0  # Example altitude
        gps_msg.undulation = 50.0  # Example undulation

        # Publish the SbgGpsPos message
        self.gps_publisher_.publish(gps_msg)
        # Example logging for GPS message
        self.get_logger().info(f"Publishing SbgGpsPos: {gps_msg}")


def main(args=None):
    rclpy.init(args=args)
    sbg_ellipse_publisher = SbgEllipsePublisher()
    while True:
        print("\nSelect the message type to publish:")
        print("1. SbgEkfEuler (Euler angles)")
        print("2. SbgGpsPos (GPS position)")
        print("3. Exit")

        choice = input("Enter your choice (1/2/3): ").strip()

        if choice == "1":
            sbg_ellipse_publisher.publish_euler_message()
        elif choice == "2":
            sbg_ellipse_publisher.publish_gps_message()
        elif choice == "3":
            print("Exiting...")
            break
        else:
            print("Invalid choice. Please enter 1, 2, or 3.")

        # rclpy.spin_once(sbg_ellipse_publisher)
        # Not really needed cause there is not subscriptions or callbacks needed to run
        rclpy.spin_once(sbg_ellipse_publisher, timeout_sec=0.1)

    # Shutdown ROS 2 when done
    sbg_ellipse_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
