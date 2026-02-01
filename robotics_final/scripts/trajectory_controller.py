#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, Pose
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import math
from tf_transformations import euler_from_quaternion


class TrajectoryController(Node):
    def __init__(self):
        super().__init__('pid_path_follower')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('linear_kp', 0.5)
        self.declare_parameter('linear_ki', 0.0)
        self.declare_parameter('linear_kd', 0.1)
        self.declare_parameter('angular_kp', 1.0)
        self.declare_parameter('angular_ki', 0.0)
        self.declare_parameter('angular_kd', 0.1)
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('control_frequency', 20.0)
        
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.linear_ki = self.get_parameter('linear_ki').value
        self.linear_kd = self.get_parameter('linear_kd').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.angular_ki = self.get_parameter('angular_ki').value
        self.angular_kd = self.get_parameter('angular_kd').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.control_frequency = self.get_parameter('control_frequency').value
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.path_sub = self.create_subscription(
            Path,
            '/global_path',
            self.on_path_received,
            10
        )
        
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.execute_control
        )
        
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        self.linear_last_error = 0.0
        self.angular_last_error = 0.0
        self.waypoint_index = 0
        self.has_path = False
        self.max_integral = 1.0
        self.trajectory = None
        
        self.get_logger().info("Trajectory controller ready and waiting for a path")
    
    def on_path_received(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Empty trajectory received; will ignore it")
            return
        
        self.trajectory = msg
        self.waypoint_index = 0
        self.has_path = True
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        self.get_logger().info(f"Loaded trajectory with {len(msg.poses)} waypoints")
    
    def fetch_robot_pose(self) -> Pose:
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose
        except TransformException:
            return None
    
    @staticmethod
    def extract_yaw(q):
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw
    
    def locate_closest_waypoint(self, robot_pose: Pose) -> int:
        if not self.trajectory or not self.trajectory.poses:
            return 0
        
        min_dist = float('inf')
        closest_idx = self.waypoint_index
        
        for i in range(self.waypoint_index, len(self.trajectory.poses)):
            dx = self.trajectory.poses[i].pose.position.x - robot_pose.position.x
            dy = self.trajectory.poses[i].pose.position.y - robot_pose.position.y
            dist = math.sqrt(dx * dx + dy * dy)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        return closest_idx
    
    def select_lookahead_waypoint(self, robot_pose: Pose, start_idx: int) -> int:
        if not self.trajectory or not self.trajectory.poses:
            return start_idx
        
        for i in range(start_idx, len(self.trajectory.poses)):
            dx = self.trajectory.poses[i].pose.position.x - robot_pose.position.x
            dy = self.trajectory.poses[i].pose.position.y - robot_pose.position.y
            dist = math.sqrt(dx * dx + dy * dy)
            
            if dist >= self.lookahead_distance:
                return i
        
        return len(self.trajectory.poses) - 1
    
    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def execute_control(self):
        if not self.has_path or not self.trajectory or not self.trajectory.poses:
            return
        
        robot_pose = self.fetch_robot_pose()
        if robot_pose is None:
            return
        
        self.waypoint_index = self.locate_closest_waypoint(robot_pose)
        target_idx = self.select_lookahead_waypoint(robot_pose, self.waypoint_index)
        
        if target_idx >= len(self.trajectory.poses):
            target_idx = len(self.trajectory.poses) - 1
        
        target_pose = self.trajectory.poses[target_idx].pose
        
        dx = target_pose.position.x - robot_pose.position.x
        dy = target_pose.position.y - robot_pose.position.y
        distance_error = math.sqrt(dx * dx + dy * dy)
        
        target_yaw = math.atan2(dy, dx)
        robot_yaw = self.extract_yaw(robot_pose.orientation)
        angular_error = self.normalize_angle(target_yaw - robot_yaw)
        
        if target_idx == len(self.trajectory.poses) - 1 and distance_error < self.goal_tolerance:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info("Destination reached; stopping the robot")
            self.has_path = False
            return
        
        dt = 1.0 / self.control_frequency
        
        self.linear_integral += distance_error * dt
        self.linear_integral = max(min(self.linear_integral, self.max_integral), -self.max_integral)
        linear_derivative = (distance_error - self.linear_last_error) / dt
        linear_output = (self.linear_kp * distance_error + 
                        self.linear_ki * self.linear_integral + 
                        self.linear_kd * linear_derivative)
        self.linear_last_error = distance_error
        
        self.angular_integral += angular_error * dt
        self.angular_integral = max(min(self.angular_integral, self.max_integral), -self.max_integral)
        angular_derivative = (angular_error - self.angular_last_error) / dt
        angular_output = (self.angular_kp * angular_error + 
                         self.angular_ki * self.angular_integral + 
                         self.angular_kd * angular_derivative)
        self.angular_last_error = angular_error
        
        cmd = Twist()
        cmd.linear.x = max(min(linear_output, self.max_linear_vel), -self.max_linear_vel)
        cmd.angular.z = max(min(angular_output, self.max_angular_vel), -self.max_angular_vel)
        
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
