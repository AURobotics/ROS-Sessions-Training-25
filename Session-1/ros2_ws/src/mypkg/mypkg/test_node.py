#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_interfaces.msg import RobotSensors
import random

class MyNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.get_logger().info(" Node Started!!")
        self.publisher_=self.create_publisher(RobotSensors,"/robot_sensors",10)
        self.create_timer(1,self.timer_callback)

    def timer_callback(self):
        msg=RobotSensors()
        msg.distance=random.randint(1,150)
        msg.ir_left=False
        msg.ir_right=True
        self.publisher_.publish(msg)

        self.get_logger().info(f"Message Published {msg.distance} , {msg.ir_left} , {msg.ir_right}")

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()