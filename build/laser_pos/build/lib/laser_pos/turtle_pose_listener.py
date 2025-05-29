#!/usr/bin/env python3

##############################
"DO NOT USE"
##############################


import rclpy
from rclpy.node import Node


class PosNodePublisher(Node):
    def __init__(self):
        super().__init__('pos_node_publisher')
        self.publisher_ = self.create_publisher(String, 'pos_now', 10)
        self.timer = self.create_timer(1.0, self.publish_position)
        self.get_logger().info("PosNodePublisher has been started.")
    
    def distanceFromObject(self):
        return input("Enter the distance from the object (imagine this is a laser): ")

def main(args=None):
    rclpy.init(args=args)
    node = PosNodePublisher()