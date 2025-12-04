#!/usr/bin/env python3
import rclpy # type: ignore
from rclpy.node import Node # type: ignore

from obstacle_manager.msg import ObstacleManagerReport

class ObstacleManager(Node):
    def __init__(self):
        super().__init__('obstacle_manager')

        self.subscriber = self.create_subscription(
            ObstacleManagerReport,
            "obstacle_manager/report_obstacle",
            self.handle_obstacle_unity_request,
            10
        )

        self.publisher = self.create_publisher(
            ObstacleManagerReport,
            "obstacle_manager/publish_obstacle",
            10
        )

        self.get_logger().info("ObstacleManager node has been started.")
    
    def handle_obstacle_unity_request(self, msg):
        self.get_logger().info(f"Received obstacle with id: {msg.id} from Unity.")
        self.publisher.publish(msg)
        self.get_logger().info(f"Published obstacle data with id: {msg.id}")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()