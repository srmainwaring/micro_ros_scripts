"""
Usage

Run agent:
micro_ros_ws % ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 --verbose 6

Run client:
RMW_IMPLEMENTATION=rmw_microxrcedds ros2 run micro_ros_demos_rclc transform_subscriber

Run publisher:
ros2 run micro_ros_scripts transform_publisher --ros-args --param rate:=10.0
"""

import math
import rclpy

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from transforms3d import euler

from geometry_msgs.msg import Transform


class TransformPublisher(Node):
    def __init__(self):
        super().__init__("transform_publisher")

        parameter_descriptor = ParameterDescriptor(description="Publish rate (Hz)")
        self.declare_parameter("rate", 1.0, parameter_descriptor)
        self.rate = self.get_parameter("rate").get_parameter_value().double_value

        self.publisher_ = self.create_publisher(
            Transform, "/geometry_msgs_msg_Transform", 10
        )
        timer_period = 1.0 / self.rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0.0
        self.y = 0.25
        self.z = 0.75
        self.yaw_deg = 0.0

    def timer_callback(self):
        yaw_rad = math.radians(self.yaw_deg)
        q = euler.euler2quat(0.0, 0.0, yaw_rad, 'ryxz')

        msg = Transform()
        msg.translation.x = self.x
        msg.translation.y = self.y
        msg.translation.z = self.z
        msg.rotation.x = q[1]
        msg.rotation.y = q[2]
        msg.rotation.z = q[3]
        msg.rotation.w = q[0]
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: {}".format(msg))
        self.x += 1.0
        self.y += 1.0
        self.z += 1.0
        self.yaw_deg += 1.0


def main(args=None):
    rclpy.init(args=args)

    transform_publisher = TransformPublisher()

    rclpy.spin(transform_publisher)

    # Destroy the node prior to exit
    transform_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
