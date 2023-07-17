"""
Usage

Run agent:
micro_ros_ws % ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 --verbose 6

Run client:
RMW_IMPLEMENTATION=rmw_microxrcedds ros2 run micro_ros_demos_rclc vector3_subscriber

Run publisher:
ros2 run micro_ros_scripts vector3_publisher --ros-args --param rate:=10.0
"""

import rclpy

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Vector3


class Vector3Publisher(Node):
    def __init__(self):
        super().__init__("vector3_publisher")

        parameter_descriptor = ParameterDescriptor(description="Publish rate (Hz)")
        self.declare_parameter("rate", 1.0, parameter_descriptor)
        self.rate = self.get_parameter("rate").get_parameter_value().double_value

        self.publisher_ = self.create_publisher(
            Vector3, "/geometry_msgs_msg_Vector3", 10
        )
        timer_period = 1.0 / self.rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0.0
        self.y = 0.25
        self.z = 0.75

    def timer_callback(self):
        msg = Vector3()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: {}".format(msg))
        self.x += 1.0
        self.y += 1.0
        self.z += 1.0


def main(args=None):
    rclpy.init(args=args)

    vector3_publisher = Vector3Publisher()

    rclpy.spin(vector3_publisher)

    # Destroy the node prior to exit
    vector3_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
