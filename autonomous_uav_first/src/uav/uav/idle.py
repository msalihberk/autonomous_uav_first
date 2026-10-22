import rclpy
from rclpy.node import Node

class UAVIdle(Node):
    def __init__(self):
        super().__init__("UAVIdle")
        

def main(args=None):
    rclpy.init(args=args)
    node = UAVIdle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()