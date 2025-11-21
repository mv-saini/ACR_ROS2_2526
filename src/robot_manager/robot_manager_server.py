#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import os
import yaml

from robot_manager.msg import RobotManagerPublisher, RobotManagerSubscriber

from ament_index_python.packages import get_package_share_directory

class RobotManager(Node):

    robotID = 1
    robots = []

    def __init__(self):
        super().__init__('robot_manager')

        pkg_share = get_package_share_directory('robot_manager')
        yaml_path = os.path.join(pkg_share, 'config', 'robotConfig.yaml')
        self.get_logger().info(f"Loading YAML file: {yaml_path}")

        self.subscriber = self.create_subscription(
            RobotManagerSubscriber,
            "robot_manager/subscribe_robot",
            self.handle_req,
            10
        )

        self.publisher = self.create_publisher(
            RobotManagerPublisher,
            "robot_manager/publish_robot",
            10
        )

        self.get_logger().info("RobotManager node has been started.")

        self.load_robot_config(yaml_path)

        # self.publish_robots()

    def handle_req(self, msg):
        self.get_logger().info(f"Robot manager received message: {msg.state}")
        if msg.state == "ready":
            self.publish_robots()
    
    def load_robot_config(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        for robot in data["robots"]:
            id = self.robotID
            robot_type = robot["robot_type"]
            move_speed =  robot["move_speed"]
            perception_radius = robot["perception_radius"]
            obstacle_distance_threshold = robot["obstacle_distance_threshold"]
            start_x = robot["start_x"]
            start_y = robot["start_y"]
            end_x = robot["end_x"]
            end_y = robot["end_y"]
            path = robot["path"]
            path_x = [point[0] for point in path]
            path_y = [point[1] for point in path]
            loop = robot["loop"]
            self.robots.append((id, robot_type, move_speed, perception_radius, obstacle_distance_threshold, start_x, start_y, end_x, end_y, path_x, path_y, loop))
            self.robotID += 1

    def publish_robots(self):
        msg = RobotManagerPublisher()

        for robot in self.robots:
            id, robot_type, move_speed, perception_radius, obstacle_distance_threshold, start_x, start_y, end_x, end_y, path_x, path_y, loop = robot
            msg.robot_id = id
            msg.robot_type = robot_type
            msg.move_speed = move_speed
            msg.perception_radius = perception_radius
            msg.obstacle_distance_threshold = obstacle_distance_threshold
            msg.start_x = start_x
            msg.start_y = start_y
            msg.end_x = end_x
            msg.end_y = end_y
            msg.path_x = path_x
            msg.path_y = path_y
            msg.loop = loop
            self.publisher.publish(msg)
            self.get_logger().info(f"Published robot with id: {id}")
        
        self.get_logger().info("Published RobotManagerPublisher message.")

def main(args=None):
    rclpy.init(args=args)
    node = RobotManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()