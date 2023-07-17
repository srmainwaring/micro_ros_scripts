"""
Usage

Run agent:
micro_ros_ws % ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 --verbose 6

Run client:
RMW_IMPLEMENTATION=rmw_microxrcedds ros2 run micro_ros_demos_rclc int32_subscriber

Run publisher:
ros2 run micro_ros_scripts int32_publisher --ros-args --param rate:=10.0
"""

import rclpy

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import Int32


class Int32Publisher(Node):
    def __init__(self, rate):
        super().__init__("int32_publisher")

        parameter_descriptor = ParameterDescriptor(description="Publish rate (Hz)")
        self.declare_parameter("rate", 1.0, parameter_descriptor)
        self.rate = self.get_parameter("rate").get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Int32, "/std_msgs_msg_Int32", 10)
        timer_period = 1.0 / self.rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int32()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: {}".format(msg))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    int32_publisher = Int32Publisher(rate=1000)

    rclpy.spin(int32_publisher)

    # Destroy the node prior to exit
    int32_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
