#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node

from path_planner.msg import PathPlannerRequest, PathPlannerResponse
from obstacle_manager.msg import ObstacleManagerObstacleReport
from robot_manager.msg import RobotManagerRobotPublisher

import heapq
from math import sqrt
import os
from ament_index_python.packages import get_package_share_directory
from concurrent.futures import ThreadPoolExecutor

from std_msgs.msg import String

class PathPlannerServer(Node):
    def __init__(self):
        super().__init__('path_planner_server')

        pkg_share = get_package_share_directory('airport_grid')
        json_path = os.path.join(pkg_share, 'config', 'airport.json')
        self.get_logger().info(f"Loading JSON map: {json_path}")

        self.graph, self.pos = self.load_graph(json_path)

        self.obstacles = dict()
        self.robots = dict()

        self.request_sub = self.create_subscription(
            PathPlannerRequest,
            "path_planner/request",
            self.handle_request,
            10
        )

        self.response_pub = self.create_publisher(
            PathPlannerResponse,
            "path_planner/response",
            10
        )
        
        self.obstacle_sub = self.create_subscription(
            ObstacleManagerObstacleReport,
            "obstacle_manager/publish_obstacle",
            self.handle_obstacle_request,
            10 
        )

        self.reset_state = self.create_subscription(
            String,
            "path_planner/reset_state",
            self.handle_reset_state,
            10
        )

        self.robot_sub = self.create_subscription(
            RobotManagerRobotPublisher,
            "robot_manager/publish_robot",
            self.handle_robot_request,
            10
        )

        self.pool = ThreadPoolExecutor(max_workers=max(2, (os.cpu_count() or 2)))

        self.get_logger().info("PathPlannerServer ready.")

    def handle_reset_state(self, msg):
        self.get_logger().info("Resetting path planner state.")
        self.obstacles = dict()
        self.robots = dict()

    def handle_robot_request(self, msg):
        self.robots[msg.robot_id] = msg.robot_type
    
    def handle_obstacle_request(self, msg):
        self.get_logger().info(f"Received obstacle with id: {msg.id} from Obstacle Manager.")
        obstacle = (msg.x, msg.y, msg.type, msg.status, msg.scale_x, msg.scale_y, msg.id)
        if(obstacle[3] == "handled"):
            if msg.id in self.obstacles:
                del self.obstacles[msg.id]
                self.get_logger().info(f"Removed obstacle with id: {msg.id}")
        elif obstacle[3] == "unhandled":
            if(msg.id not in self.obstacles):
                self.obstacles[msg.id] = obstacle
                self.get_logger().info(f"Added new obstacle with id: {msg.id}")

    def load_graph(self, path):
        with open(path, 'r') as f:
            data = json.load(f)

        graph = {}
        pos = {}
        node_types = {}

        for n in data["nodes"]:
            nid = int(n["id"])
            if n["type"] in [1, 2, 3]:
                pos[nid] = (float(n["x"]), float(n["y"]))
                node_types[nid] = n["type"]
                neighbors = [int(x) for x in n.get("neighbors", [])]
                valid_neighbors = []
                for neighbor in neighbors:
                    neighbor_node = next((node for node in data["nodes"] if int(node["id"]) == neighbor), None)
                    if neighbor_node and neighbor_node["type"] in [1, 2, 3]:
                        valid_neighbors.append(neighbor)
                graph[nid] = valid_neighbors

        self.node_types = node_types
        self.get_logger().info(f"Loaded graph with {len(graph)} nodes.")

        return graph, pos

    def euclidean_distance(self, a, b):
        (x1, y1) = self.pos[a]
        (x2, y2) = self.pos[b]
        return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def closest_node(self, x, y):
        best = None
        best_d = float("inf")

        for nid, (nx, ny) in self.pos.items():
            d = (x - nx) ** 2 + (y - ny) ** 2
            if d < best_d:
                best_d = d
                best = nid

        return best

    def a_star(self, start, goal, robot_id):        
        blocked_nodes = set()
        active_obstacles = []
        
        # Determine blocked nodes
        for obs in self.obstacles.values():
            obs_x, obs_y, obs_type, obs_status, obs_scale_x, obs_scale_y, obs_id = obs
            if(obs_status == "unhandled"):
                if((obs_type == "DirtObstacle" and self.robots.get(robot_id, "") == "cleaner") or
                   (obs_type == "UnattendedObstacle" and self.robots.get(robot_id, "") == "security")):
                    continue
                active_obstacles.append(obs)
                for nid, (nx, ny) in self.pos.items():
                    if abs(nx - obs_x) <= obs_scale_x and abs(ny - obs_y) <= obs_scale_y:
                        blocked_nodes.add(nid)

        # Adjust goal if blocked
        actual_goal = goal
        if goal in blocked_nodes:
            neighbors = self.graph.get(goal, [])
            min_dist = float("inf")
            for neighbor in neighbors:
                if neighbor not in blocked_nodes:
                    dist = self.euclidean_distance(start, neighbor)
                    if dist < min_dist:
                        min_dist = dist
                        actual_goal = neighbor

        open_set = [(0, start)]
        came_from = {}
        g = {start: 0}

        while open_set:
            _, cur = heapq.heappop(open_set)

            if cur == actual_goal:
                return self.reconstruct_path(came_from, cur)

            cur_pos = self.pos[cur]

            for neighbor in self.graph[cur]:
                if neighbor in blocked_nodes:
                    continue

                neighbor_pos = self.pos[neighbor]

                # Base cost
                node_type = self.get_node_type(neighbor)
                extra_cost = 0
                if node_type == 1:
                    extra_cost = 1.0
                elif node_type == 3:
                    extra_cost = 2.0
                
                # Side cost
                side_penalty = 0.0
                
                for obs in active_obstacles:
                    obs_x, obs_y, _, _, obs_sx, obs_sy, _ = obs
                    
                    dist_to_obs = sqrt((cur_pos[0] - obs_x)**2 + (cur_pos[1] - obs_y)**2)
                    
                    influence_radius = max(obs_sx, obs_sy) * 4.0 
                    
                    if dist_to_obs < influence_radius:
                        vec_rob_obs_x = obs_x - cur_pos[0]
                        vec_rob_obs_y = obs_y - cur_pos[1]
                        
                        vec_move_x = neighbor_pos[0] - cur_pos[0]
                        vec_move_y = neighbor_pos[1] - cur_pos[1]
                        
                        cross_prod = (vec_move_x * vec_rob_obs_y) - (vec_move_y * vec_rob_obs_x)

                        if cross_prod < -0.01:
                            side_penalty += 20.0 
                        elif cross_prod > 0.01:
                            side_penalty += 0.0

                cost = g[cur] + self.euclidean_distance(cur, neighbor) + extra_cost + side_penalty
                
                if neighbor not in g or cost < g[neighbor]:
                    g[neighbor] = cost
                    f = cost + self.euclidean_distance(neighbor, actual_goal)
                    heapq.heappush(open_set, (f, neighbor))
                    came_from[neighbor] = cur

        return None

    def get_node_type(self, node_id):
        return self.node_types.get(node_id, 2)
    
    def reconstruct_path(self, came_from, cur):
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        return list(reversed(path))

    def handle_request(self, req):
        robot_id = req.robot_id

        start = self.closest_node(req.start_x, req.start_y)
        goal = self.closest_node(req.end_x, req.end_y)

        self.get_logger().info(
            f"[Robot {robot_id}] request: Unity({req.start_x},{req.start_y}) → "
            f"Unity({req.end_x},{req.end_y}) | Graph nodes: {start} → {goal}"
        )

        self.pool.submit(
            self._plan_and_publish,
            robot_id, start, goal,
            req.start_x, req.start_y,
            req.end_x, req.end_y,
        )
    
    def _plan_and_publish(self, robot_id, start_node, goal_node, start_x, start_y, end_x, end_y):
        node_path = self.a_star(start_node, goal_node, robot_id)

        res = PathPlannerResponse()
        res.robot_id = robot_id

        if node_path is None:
            res.success = False
            res.path_x = []
            res.path_y = []
            self.get_logger().warn(f"[Robot {robot_id}] No path found.")
        else:
            res.success = True
            for nid in node_path:
                (x, y) = self.pos[nid]
                res.path_x.append(x)
                res.path_y.append(y)

        self.response_pub.publish(res)
    
    def destroy_node(self):
        self.pool.shutdown(wait=True)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()