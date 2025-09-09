import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial

class MyClient(Node):
    def __init__(self):
        super().__init__("client_node")
        self.call_service_add_ints(6,7)

    def call_service_add_ints(self,a,b):
        client=self.create_client(AddTwoInts,"add_ints")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting...")

        request=AddTwoInts.Request()
        request.a=a
        request.b=b

        future=client.call_async(request)
        future.add_done_callback(partial(self.add_callback,a=a,b=b)) # this will call self.callback when service has replied

    def add_callback(self,future,a,b):
        try:
            response=future.result()
            self.get_logger().info(f"Result of add_ints({a}, {b}) = {response.sum}")
        except Exception as e:
            self.get_logger().error("Service call failed: %r" %(e,))


def main():
    rclpy.init()
    node = MyClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()