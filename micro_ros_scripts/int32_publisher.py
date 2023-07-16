import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class Int32Publisher(Node):
    def __init__(self):
        super().__init__("int32_publisher")
        self.publisher_ = self.create_publisher(Int32, "/std_msgs_msg_Int32", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int32()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    int32_publisher = Int32Publisher()

    rclpy.spin(int32_publisher)

    # Destroy the node prior to exit
    int32_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
