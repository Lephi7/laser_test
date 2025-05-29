#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class TurtleFollower(Node):
    def __init__(self):
        super().__init__('turtle_follower')
        self.declare_parameter('desired_distance', 0.2)
        self.desired_distance = self.get_parameter('desired_distance').value

        self.turtle_x = None
        self.object_x = None

        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.object_sub = self.create_subscription(Float32, 'object_x', self.object_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.turtle_x = msg.x

    def object_callback(self, msg):
        self.object_x = msg.data

    def control_loop(self):
        # Update parameter in case it was changed at runtime
        self.desired_distance = self.get_parameter('desired_distance').value

        if self.turtle_x is None or self.object_x is None:
            return  # Wait for both positions

        # Calculate distance error (positive if turtle is too far to the left)
        error = (self.object_x - self.turtle_x) - self.desired_distance

        # Proportional control
        k_p = 1.1
        cmd = Twist()
        cmd.linear.x = k_p * error
        cmd.angular.z = 0.0

        # Clamp speed
        max_speed = 0.2
        cmd.linear.x = max(min(cmd.linear.x, max_speed), -max_speed)

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f"Turtle x: {self.turtle_x:.4f}, Object x: {self.object_x:.4f}, Error: {error:.4f}, Cmd: {cmd.linear.x:.4f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TurtleFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# This code is a simple ROS2 node that allows a turtle to follow an arbitrary distance from an object.
# It uses a proportional control approach to adjust the turtle's speed based on the distance error.
    
