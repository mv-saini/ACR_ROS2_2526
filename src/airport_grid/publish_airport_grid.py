#!/usr/bin/env python3
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
import json
import os

from ament_index_python.packages import get_package_share_directory # type: ignore

from std_msgs.msg import String # type: ignore

class AirportGridPublisher(Node):

    def __init__(self):
        super().__init__("airport_grid_publisher")

        pkg_share = get_package_share_directory('airport_grid')
        json_path = os.path.join(pkg_share, 'config', 'airport.json')
        self.get_logger().info(f"Loading JSON file: {json_path}")

        self.load_config(json_path)

        self.publisher = self.create_publisher(String, "airport_grid/response_airport_grid", 10)
        self.subscriber = self.create_subscription(String, "airport_grid/request_airport_grid", self.handle_request, 10)

        self.get_logger().info("AirportGridPublisher node has been started.")

    def load_config(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
        
        self.grid_data = data
        self.get_logger().info("Airport grid configuration loaded successfully.")

    def handle_request(self, msg):
        """Handle incoming requests for the airport grid."""
        self.get_logger().info("Received request for airport grid.")
        self.publish_grid()

    def publish_grid(self):
        """Send YAML → JSON to Unity."""

        msg = String()
        msg.data = json.dumps(self.grid_data)

        self.publisher.publish(msg)
        self.get_logger().info("Published airport grid to /response_airport_grid")


def main():
    rclpy.init()
    node = AirportGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
