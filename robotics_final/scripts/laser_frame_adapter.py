#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ScanFrameAdapter(Node):
    def __init__(self):
        super().__init__('scan_frame_adapter')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.scan_publisher = self.create_publisher(
            LaserScan,
            '/scan',
            qos_profile
        )
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/gz_lidar/scan',
            self.handle_scan_data,
            qos_profile
        )
        
        self.get_logger().info("Scan frame adapter initialized; forwarding from /gz_lidar/scan to /scan")
    
    def handle_scan_data(self, incoming_scan: LaserScan):
        adapted_scan = LaserScan()
        adapted_scan.header = incoming_scan.header
        adapted_scan.header.frame_id = 'rplidar_c1'
        adapted_scan.angle_min = incoming_scan.angle_min
        adapted_scan.angle_max = incoming_scan.angle_max
        adapted_scan.angle_increment = incoming_scan.angle_increment
        adapted_scan.time_increment = incoming_scan.time_increment
        adapted_scan.scan_time = incoming_scan.scan_time
        adapted_scan.range_min = incoming_scan.range_min
        adapted_scan.range_max = incoming_scan.range_max
        adapted_scan.ranges = incoming_scan.ranges
        adapted_scan.intensities = incoming_scan.intensities
        
        self.scan_publisher.publish(adapted_scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
