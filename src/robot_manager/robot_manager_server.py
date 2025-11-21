#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import os
import yaml

from robot_manager.msg import RobotManagerRobotPublisher, RobotManagerRobotSubscriber, RobotManagerTrackerSubscriber

from ament_index_python.packages import get_package_share_directory

class RobotManager(Node):

    robotID = 1
    robots = {}

    def __init__(self):
        super().__init__('robot_manager')

        pkg_share = get_package_share_directory('robot_manager')
        yaml_path = os.path.join(pkg_share, 'config', 'robotConfig.yaml')
        self.get_logger().info(f"Loading YAML file: {yaml_path}")

        self.robotSubscriber = self.create_subscription(
            RobotManagerRobotSubscriber,
            "robot_manager/subscribe_robot",
            self.handle_robot_req,
            10
        )

        self.robotPublisher = self.create_publisher(
            RobotManagerRobotPublisher,
            "robot_manager/publish_robot",
            10
        )

        self.trackerSubscriber = self.create_subscription(
            RobotManagerTrackerSubscriber,
            "robot_manager/subscribe_tracker",
            self.handle_tracker_req,
            10
        )

        self.get_logger().info("RobotManager node has been started.")
        self.load_robot_config(yaml_path)
        # self.publish_robots()

    def handle_robot_req(self, msg):
        self.get_logger().info(f"Robot manager received message: {msg.state}")
        if msg.state == "ready":
            self.publish_robots()

    def handle_tracker_req(self, msg):
        self.get_logger().info(f"Robot manager received tracker from: {msg.robot_id}")
        self.robots[msg.robot_id] = (msg.robot_id, msg.robot_type, msg.move_speed, msg.perception_radius, msg.obstacle_distance_threshold, 
                                     msg.current_x, msg.current_y, msg.start_x, msg.start_y, msg.end_x, msg.end_y, msg.destinations_x, msg.destinations_y, msg.loop, 
                                     msg.obstacle_detected, msg.performing_task)
        self.get_logger().info(f"Updated robot {msg.robot_id}")
    
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
            current_x = start_x
            current_y = start_y
            # self.robots.append((id, robot_type, move_speed, perception_radius, obstacle_distance_threshold, start_x, start_y, end_x, end_y, path_x, path_y, loop))
            self.robots[id] = (id, robot_type, move_speed, perception_radius, obstacle_distance_threshold, current_x, current_y, start_x, start_y, end_x, end_y, 
                               path_x, path_y, loop, False, False)
            self.robotID += 1

    def publish_robots(self):
        msg = RobotManagerRobotPublisher()

        for robot in self.robots.values():
            (robot_id, robot_type, move_speed, perception_radius, obstacle_distance_threshold,
             current_x, current_y, start_x, start_y, end_x, end_y, path_x, path_y, loop,
             obstacle_detected, performing_task) = robot
            msg.robot_id = robot_id
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
            self.robotPublisher.publish(msg)
            self.get_logger().info(f"Published robot with id: {id}")
        
        self.get_logger().info("Published RobotManagerRobotPublisher message.")

def main(args=None):
    rclpy.init(args=args)
    node = RobotManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()