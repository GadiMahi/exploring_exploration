# exploring_exploration/test_frontier_node.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from gps_frontier_explorer.frontier_detection import detect_frontier_cells, grid_to_world


class TestFrontierNode(Node):
    def __init__(self):
        super().__init__('test_frontier_node')
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

    def map_callback(self, msg):
        cells = detect_frontier_cells(msg)
        self.get_logger().info(f"Detected {len(cells)} frontier cells")
        if cells:
            # Show first 5 frontier cells in world coords
            for col, row in cells[:5]:
                wx, wy = grid_to_world(msg, col, row)
                self.get_logger().info(f"Frontier ({col},{row}) → ({wx:.2f}, {wy:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = TestFrontierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
