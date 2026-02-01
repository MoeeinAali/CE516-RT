#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class WheelCommandInterface(Node):
    def __init__(self):
        super().__init__('motor_command_node')
        
        self.motor_sub = self.create_subscription(
            Float64MultiArray,
            '/motor_commands',
            self.process_commands,
            10
        )
        
        self.left_motor_pub = self.create_publisher(Float64, '/left_motor_rpm', 10)
        self.right_motor_pub = self.create_publisher(Float64, '/right_motor_rpm', 10)
        
        self.get_logger().info("Wheel motor interface active and awaiting commands.")
    
    def process_commands(self, msg: Float64MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn("Received motor command with insufficient elements; ignoring")
            return
        
        left_msg = Float64()
        right_msg = Float64()
        
        left_msg.data = msg.data[0]
        right_msg.data = msg.data[1]
        
        self.left_motor_pub.publish(left_msg)
        self.right_motor_pub.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelCommandInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
