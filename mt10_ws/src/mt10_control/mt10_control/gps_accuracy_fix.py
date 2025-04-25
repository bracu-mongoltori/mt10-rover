#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sbg_driver.msg import SbgGpsPos
from ublox_msgs.msg import NavPVT


class DualGpsListener(Node):

    def __init__(self):
        super().__init__('dual_gps_listener')

        # Subscriber to /sbg/gps_pos
        self.subscription_gps_pos = self.create_subscription(
            SbgGpsPos,
            '/sbg/gps_pos',
            self.gps_pos_callback,
            10
        )
        self.subscription_gps_pos  # prevent unused variable warning

        # Subscriber to /navpvt
        self.subscription_navpvt = self.create_subscription(
            NavPVT,
            '/navpvt',
            self.navpvt_callback,
            10
        )
        self.subscription_navpvt  # prevent unused variable warning

        self.get_logger().info('Dual GPS listener node started.')
        self.sparkfun_msg = NavPVT()
        self.publisher = self.create_publisher(SbgGpsPos, '/best_gps_acc', 10)

    def gps_pos_callback(self, msg: SbgGpsPos):
        accuracy_sbg_x = msg.position_accuracy.x
        accuracy_sbg_y = msg.position_accuracy.y
        accuracy_sparkfun_x = self.sparkfun_msg.h_acc / 1000.0
        accuracy_sparkfun_x = self.sparkfun_msg.h_acc / 1000.0

        if accuracy_sbg_x <= accuracy_sparkfun_x:
            self.publisher.publish(msg)
        else:
            msg.latitude = self.sparkfun_msg.lat / 1e-7
            








    def navpvt_callback(self, msg: NavPVT):
        self.sparkfun_msg = msg


def main(args=None):
    rclpy.init(args=args)
    node = DualGpsListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
