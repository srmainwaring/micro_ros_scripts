"""
Usage

Run publisher:
ros2 run micro_ros_scripts joy_publisher --ros-args --param rate:=10.0
"""

import rclpy

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Joy


class JoyPublisher(Node):
    def __init__(self):
        super().__init__("joy_publisher")

        parameter_descriptor = ParameterDescriptor(description="Publish rate (Hz)")
        self.declare_parameter("rate", 1.0, parameter_descriptor)
        self.rate = self.get_parameter("rate").get_parameter_value().double_value

        self.publisher_ = self.create_publisher(
            Joy, "/ap/joy", 10
        )
        timer_period = 1.0 / self.rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.size = 32
        self.axes_value = 0.0

    def timer_callback(self):
        msg = Joy()
        for i in range(self.size):
            msg.axes.append(self.axes_value + float(i))
            msg.buttons.append(int(i))
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: {}".format(msg))
        self.axes_value += 1.0


def main(args=None):
    rclpy.init(args=args)

    joy_publisher = JoyPublisher()

    rclpy.spin(joy_publisher)

    # Destroy the node prior to exit
    joy_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
