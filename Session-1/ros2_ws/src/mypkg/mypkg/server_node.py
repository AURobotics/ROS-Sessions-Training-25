#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
class MyServer(Node):
    def __init__(self):
        super().__init__("server_node")
        self.server=self.create_service(AddTwoInts,"add_ints",self.service_callback)

    def service_callback(self,request,response):
        response.sum = request.a + request.b
        return response
    
def main():
    rclpy.init()
    node = MyServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()