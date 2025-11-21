#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from path_planner.msg import PathPlannerRequest, PathPlannerResponse
from obstacle_manager.msg import ObstacleManagerPublisher

import yaml
import heapq
from math import sqrt
import os
from ament_index_python.packages import get_package_share_directory
from concurrent.futures import ThreadPoolExecutor


class PathPlannerServer(Node):
    def __init__(self):
        super().__init__('path_planner_server')

        pkg_share = get_package_share_directory('path_planner')
        yaml_path = os.path.join(pkg_share, 'config', 'map.yaml')
        self.get_logger().info(f"Loading YAML map: {yaml_path}")

        self.graph, self.pos = self.load_graph(yaml_path)

        self.obstacles = list()

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
            ObstacleManagerPublisher,
            "obstacle_manager/publish_obstacle",
            self.handle_obstacle_request,
            10 
        )

        self.pool = ThreadPoolExecutor(max_workers=max(2, (os.cpu_count() or 2)))

        self.get_logger().info("PathPlannerServer ready.")
    
    def handle_obstacle_request(self, msg):
        self.get_logger().info(f"Received obstacle data: x={msg.x}, y={msg.y}, type={msg.type}, id={msg.id}")
        obstacle = (msg.x, msg.y, msg.type, msg.id)
        if(obstacle not in self.obstacles):
            self.obstacles.append(obstacle)
            self.get_logger().info(f"Added obstacle id: {msg.id}")

    def load_graph(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        graph = {}
        pos = {}

        # load nodes
        for n in data["nodes"]:
            nid = int(n["id"])
            pos[nid] = (float(n["x"]), float(n["y"]))

        # load adjacency
        for nid, neighbors in data["adjacency"].items():
            graph[int(nid)] = [int(x) for x in neighbors]

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

    def a_star(self, start, goal):
        open_set = [(0, start)]
        came_from = {}
        g = {start: 0}

        while open_set:
            _, cur = heapq.heappop(open_set)

            if cur == goal:
                return self.reconstruct_path(came_from, cur)

            for neighbor in self.graph[cur]:
                cost = g[cur] + self.euclidean_distance(cur, neighbor)
                if neighbor not in g or cost < g[neighbor]:
                    g[neighbor] = cost
                    f = cost + self.euclidean_distance(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))
                    came_from[neighbor] = cur

        return None

    def reconstruct_path(self, came_from, cur):
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        return list(reversed(path))

    def handle_request(self, req):
        robot_id = req.robot_id

        # convert unity coords → closest graph nodes
        start = self.closest_node(req.start_x, req.start_y)
        goal = self.closest_node(req.end_x, req.end_y)

        self.get_logger().info(
            f"[Robot {robot_id}] request: Unity({req.start_x},{req.start_y}) → "
            f"Unity({req.end_x},{req.end_y})  | Graph nodes: {start} → {goal}"
        )

        self.pool.submit(
            self._plan_and_publish,
            robot_id, start, goal,
            req.start_x, req.start_y,
            req.end_x, req.end_y,
        )
    
    def _plan_and_publish(self, robot_id, start_node, goal_node, start_x, start_y, end_x, end_y):
        node_path = self.a_star(start_node, goal_node)

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

        # publish response
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
