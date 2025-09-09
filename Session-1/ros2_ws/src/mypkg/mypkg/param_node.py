import rclpy
from rclpy.node import Node

class ParamNode(Node):
    def __init__(self):
        super().__init__("param_node")

        #Step 1: Declare Parameters with Default values
        self.declare_parameter("robot_name","default_name")
        self.declare_parameter("robot_speed",1.0)
        self.declare_parameter("manual_mode",True)

        #Retrieve Values (After Overrides from YAML/CLI)
        robot_name=self.get_parameter("robot_name").value
        robot_speed = self.get_parameter("robot_speed").value
        manual_mode = self.get_parameter("manual_mode").value

        self.get_logger().info(f"Robot Name: {robot_name}")
        self.get_logger().info(f"Robot Speed: {robot_speed}")
        self.get_logger().info(f"Manual Mode: {manual_mode}")

def main():
    rclpy.init()
    node=ParamNode()
    rclpy.spin(node)
    rclpy.shutdown()

