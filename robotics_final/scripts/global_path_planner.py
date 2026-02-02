#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from robotics_final.srv import PlanPath
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import math
import heapq
from typing import Optional, List, Tuple


class SearchNode:
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
        self.g_cost = float('inf')
        self.f_cost = float('inf')
        self.parent: Optional['SearchNode'] = None
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost


class GlobalPathPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.on_map_received,
            10
        )
        
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.on_pose_update,
            10
        )
        
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        
        self.plan_service = self.create_service(
            PlanPath,
            '/plan_path',
            self.handle_plan_request
        )
        
        self.grid_map: Optional[OccupancyGrid] = None
        self.robot_pose: Optional[Pose] = None
        self.map_available = False
        self.pose_available = False
        
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.grid_width = 0
        self.grid_height = 0
        
        self.get_logger().info("A* path planner node initialized")
    
    def on_map_received(self, msg: OccupancyGrid):
        self.grid_map = msg
        self.map_available = True
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.grid_width = msg.info.width
        self.grid_height = msg.info.height
    
    def on_pose_update(self, msg: PoseWithCovarianceStamped):
        self.robot_pose = msg.pose.pose
        self.pose_available = True
    
    def to_grid_coords(self, wx: float, wy: float) -> Optional[Tuple[int, int]]:
        if not self.map_available:
            return None
        
        gx = int((wx - self.map_origin_x) / self.map_resolution)
        gy = int((wy - self.map_origin_y) / self.map_resolution)
        
        if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
            return (gx, gy)
        return None
    
    def to_world_coords(self, gx: int, gy: int) -> Tuple[float, float]:
        wx = gx * self.map_resolution + self.map_origin_x + self.map_resolution / 2.0
        wy = gy * self.map_resolution + self.map_origin_y + self.map_resolution / 2.0
        return (wx, wy)
    
    def is_traversable(self, x: int, y: int) -> bool:
        if not self.map_available or x < 0 or x >= self.grid_width or y < 0 or y >= self.grid_height:
            return False
        
        index = y * self.grid_width + x
        if index >= len(self.grid_map.data):
            return False
        
        cost = self.grid_map.data[index]
        return cost == 0 or cost == -1
    
    def compute_heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        # Euclidean distance
        dx = x2 - x1
        dy = y2 - y1
        return math.sqrt(dx * dx + dy * dy)
    
    def expand_neighbors(self, x: int, y: int) -> List[Tuple[int, int]]:
        neighbors = []
        # Check adjacent cells first (4-connectivity), then diagonals
        directions = [
            (0, 1), (0, -1), (1, 0), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if self.is_traversable(nx, ny):
                neighbors.append((nx, ny))
        
        return neighbors
    
    def compute_path(self, start: Pose, goal: Pose) -> Path:
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        if not self.map_available:
            self.get_logger().warn("Map data not available yet; skipping path computation")
            return path
        
        start_coords = self.to_grid_coords(start.position.x, start.position.y)
        if start_coords is None:
            self.get_logger().error("Start position is outside the map boundaries")
            return path
        start_x, start_y = start_coords
        
        goal_coords = self.to_grid_coords(goal.position.x, goal.position.y)
        if goal_coords is None:
            self.get_logger().error("Goal position is outside the map boundaries")
            return path
        goal_x, goal_y = goal_coords
        
        open_set = []
        node_registry = {}
        cost_map = {}
        
        start_node = SearchNode(start_x, start_y)
        start_node.g_cost = 0.0
        start_node.f_cost = self.compute_heuristic(start_x, start_y, goal_x, goal_y)
        heapq.heappush(open_set, start_node)
        node_registry[(start_x, start_y)] = start_node
        cost_map[(start_x, start_y)] = 0.0
        
        goal_node = None
        iterations = 0
        MAX_ITERATIONS = 100000
        
        while open_set and iterations < MAX_ITERATIONS:
            iterations += 1
            current = heapq.heappop(open_set)
            
            if current.x == goal_x and current.y == goal_y:
                goal_node = current
                break
            
            for nx, ny in self.expand_neighbors(current.x, current.y):
                dx = nx - current.x
                dy = ny - current.y
                move_cost = math.sqrt(2.0) if (dx != 0 and dy != 0) else 1.0
                tentative_g = current.g_cost + move_cost
                
                if (nx, ny) not in cost_map or tentative_g < cost_map[(nx, ny)]:
                    cost_map[(nx, ny)] = tentative_g
                    
                    if (nx, ny) not in node_registry:
                        node_registry[(nx, ny)] = SearchNode(nx, ny)
                    
                    neighbor_node = node_registry[(nx, ny)]
                    neighbor_node.g_cost = tentative_g
                    neighbor_node.f_cost = tentative_g + self.compute_heuristic(nx, ny, goal_x, goal_y)
                    neighbor_node.parent = current
                    heapq.heappush(open_set, neighbor_node)
        
        if iterations >= MAX_ITERATIONS:
            self.get_logger().warn("A* search reached the iteration limit and stopped")
        
        if goal_node:
            path_nodes = []
            current = goal_node
            while current is not None:
                path_nodes.append(current)
                current = current.parent
            path_nodes.reverse()
            
            for node in path_nodes:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                
                wx, wy = self.to_world_coords(node.x, node.y)
                pose.pose.position.x = wx
                pose.pose.position.y = wy
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                
                path.poses.append(pose)
            
            self.get_logger().info(f"Path computed successfully ({len(path.poses)} waypoints)")
        else:
            self.get_logger().warn("No valid path could be computed between the start and goal")
        
        return path
    
    def handle_plan_request(self, request, response):
        self.get_logger().info("Plan path service request received")
        self.get_logger().info(
            f"Requested goal coordinates: ({request.goal.pose.position.x:.2f}, "
            f"{request.goal.pose.position.y:.2f})"
        )
        
        if not self.map_available:
            self.get_logger().error("Cannot plan path: map data is not present")
            response.path.header.frame_id = 'map'
            response.path.header.stamp = self.get_clock().now().to_msg()
            return response
        
        start_pose = Pose()
        
        if self.pose_available:
            start_pose = self.robot_pose
            self.get_logger().info(
                f"Using AMCL-derived start pose: ({start_pose.position.x:.2f}, "
                f"{start_pose.position.y:.2f})"
            )
        else:
            self.get_logger().warn("AMCL pose not available; attempting TF lookup for robot pose")
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time()
                )
                
                start_pose.position.x = transform.transform.translation.x
                start_pose.position.y = transform.transform.translation.y
                start_pose.position.z = transform.transform.translation.z
                start_pose.orientation = transform.transform.rotation
                
                self.get_logger().info(
                    f"Using TF-derived start pose: ({start_pose.position.x:.2f}, "
                    f"{start_pose.position.y:.2f})"
                )
            except TransformException as ex:
                self.get_logger().error(f"Failed to obtain robot pose from TF: {ex}")
                response.path.header.frame_id = 'map'
                response.path.header.stamp = self.get_clock().now().to_msg()
                return response
        
        self.get_logger().info(
            f"Computing path from ({start_pose.position.x:.2f}, {start_pose.position.y:.2f}) "
            f"to ({request.goal.pose.position.x:.2f}, {request.goal.pose.position.y:.2f})"
        )
        
        response.path = self.compute_path(start_pose, request.goal.pose)
        
        if response.path.poses:
            self.path_pub.publish(response.path)
            self.get_logger().info(
                f"Published path to /global_path ({len(response.path.poses)} waypoints)"
            )
        else:
            self.get_logger().warn("Planner returned an empty path (no valid route found)")
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
