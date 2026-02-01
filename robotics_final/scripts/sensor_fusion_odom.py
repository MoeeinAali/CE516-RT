#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class OdometryFusionFilter(Node):
    def __init__(self):
        super().__init__('ekf_diff_imu')
        
        self.declare_parameter('wheel_odom_topic', '/wheel_encoder/odom')
        self.declare_parameter('odom_topic', '/ekf_diff_imu/odom')
        self.declare_parameter('imu_topic', '/zed/zed_node/imu/data_raw')
        self.declare_parameter('sigma_v', 0.10)
        self.declare_parameter('sigma_omega', 1e-8)
        
        self.wheel_odom_topic = self.get_parameter('wheel_odom_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.sigma_v = self.get_parameter('sigma_v').value
        self.sigma_omega = math.sqrt(self.get_parameter('sigma_omega').value)
        
        self.state_vector = np.zeros(3)  # [x, y, theta]
        self.covariance_matrix = np.eye(3) * 0.1
        self.last_predict_time = None
        self.last_v = 0.0
        self.last_omega = 0.0
        
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.wheel_odom_sub = self.create_subscription(
            Odometry,
            self.wheel_odom_topic,
            self.handle_wheel_odom,
            qos_profile
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.handle_imu_data,
            qos_profile
        )
        
        self.get_logger().info("EKF odometry fusion node started. Subscribing to wheel and IMU topics.")
    
    @staticmethod
    def normalize_angle(angle):
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        while angle > math.pi:
            angle -= 2.0 * math.pi
        return angle
    
    def propagate_state(self, v, omega, dt):
        th = self.state_vector[2]
        c = math.cos(th)
        s = math.sin(th)
        
        self.state_vector[0] += v * c * dt
        self.state_vector[1] += v * s * dt
        self.state_vector[2] = self.normalize_angle(th + omega * dt)
        
        F = np.eye(3)
        F[0, 2] = -v * s * dt
        F[1, 2] = v * c * dt
        
        G = np.array([
            [c * dt, 0.0],
            [s * dt, 0.0],
            [0.0, dt]
        ])
        
        Qu = np.diag([self.sigma_v ** 2, self.sigma_omega ** 2])
        Q = G @ Qu @ G.T
        
        self.covariance_matrix = F @ self.covariance_matrix @ F.T + Q
    
    def correct_with_yaw(self, yaw_meas, var_yaw):
        H = np.array([[0.0, 0.0, 1.0]])
        
        z_pred = self.state_vector[2]
        y = self.normalize_angle(yaw_meas - z_pred)
        
        S = (H @ self.covariance_matrix @ H.T)[0, 0] + var_yaw
        K = (self.covariance_matrix @ H.T) / S
        
        self.state_vector += K.flatten() * y
        self.state_vector[2] = self.normalize_angle(self.state_vector[2])
        
        I = np.eye(3)
        self.covariance_matrix = (I - K @ H) @ self.covariance_matrix
    
    def broadcast_odometry(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.state_vector[0]
        odom.pose.pose.position.y = self.state_vector[1]
        odom.pose.pose.position.z = 0.0
        
        q = quaternion_from_euler(0.0, 0.0, self.state_vector[2])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = self.covariance_matrix[0, 0]
        odom.pose.covariance[7] = self.covariance_matrix[1, 1]
        odom.pose.covariance[35] = self.covariance_matrix[2, 2]
        
        odom.twist.twist.linear.x = self.last_v
        odom.twist.twist.angular.z = self.last_omega
        
        self.odom_pub.publish(odom)
        
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.state_vector[0]
        tf_msg.transform.translation.y = self.state_vector[1]
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(tf_msg)
    
    def handle_wheel_odom(self, msg: Odometry):
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        t = msg.header.stamp
        
        if self.last_predict_time is None:
            self.last_predict_time = t
        
        dt = (t.sec - self.last_predict_time.sec) + (t.nanosec - self.last_predict_time.nanosec) * 1e-9
        if dt <= 1e-4:
            dt = 1e-4
        
        self.propagate_state(v, omega, dt)
        self.last_v = v
        self.last_omega = omega
        
        self.broadcast_odometry(t)
        self.last_predict_time = t
    
    def handle_imu_data(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        var_yaw = 1e-8
        if msg.orientation_covariance[8] >= 0.0:
            var_yaw = max(msg.orientation_covariance[8], 1e-8)
        
        self.correct_with_yaw(yaw, var_yaw)
        self.broadcast_odometry(msg.header.stamp)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryFusionFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
