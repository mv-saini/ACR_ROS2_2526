#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from obstacle_manager.msg import ObstacleManagerSubscriber, ObstacleManagerPublisher

import time

class ObstacleManager(Node):
    def __init__(self):
        super().__init__('obstacle_manager')

        self.subscriber = self.create_subscription(
            ObstacleManagerSubscriber,
            "obstacle_manager/report_obstacle",
            self.handle_obstacle_unity_request,
            10
        )

        self.publisher = self.create_publisher(
            ObstacleManagerPublisher,
            "obstacle_manager/publish_obstacle",
            10
        )

        self.get_logger().info("ObstacleManager node has been started.")
    
    def handle_obstacle_unity_request(self, msg):
        self.get_logger().info(f"Received obstacle data: x={msg.x}, y={msg.y}, type={msg.type}")

        response_msg = ObstacleManagerPublisher()
        response_msg.x = msg.x
        response_msg.y = msg.y
        response_msg.type = msg.type
        response_msg.id = f"obstacle_{time.time()}"

        self.publisher.publish(response_msg)
        self.get_logger().info(f"Published obstacle data with id: {response_msg.id}")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()