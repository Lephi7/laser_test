import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ObjectPosition(Node):
    def __init__(self):
        super().__init__('object_position')
        self.publisher_ = self.create_publisher(Float32, 'object_x', 10)
        self.timer = self.create_timer(0.5, self.publish_position)
        self.x_pos = 0.0  # Default position

    def publish_position(self):
        try:
            user_input = input(f"Enter new x position for object (current: {self.x_pos}): ")
            self.x_pos = float(user_input)
        except ValueError:
            self.get_logger().error("Invalid input. Please enter a number.")
            return

        msg = Float32()
        msg.data = self.x_pos
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published object x position: {self.x_pos}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectPosition()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()