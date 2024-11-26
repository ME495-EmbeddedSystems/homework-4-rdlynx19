"""Autonomous exploration of the environment."""
import rclpy
from rclpy.node import Node

class Explore(Node):
    """Node to explore the environment."""

    def __init__(self):
        """Initialise member variables of the class."""


def main(args=None):
    """Spin the explore node."""
    rclpy.init(args=args)
    node = Explore()
    rclpy.spin(node)
    rclpy.shutdown()

