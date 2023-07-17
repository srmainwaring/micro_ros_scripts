"""
Usage

Run agent:
micro_ros_ws % ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 --verbose 6

Run client:
RMW_IMPLEMENTATION=rmw_microxrcedds ros2 run micro_ros_demos_rclc tfmessage_subscriber

Run publisher:
ros2 run micro_ros_scripts tfmessage_publisher --ros-args --param rate:=10.0
"""

import math
import rclpy

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from transforms3d import euler

from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


class TFMessagePublisher(Node):
    def __init__(self, rate):
        super().__init__("tf_message_publisher")

        parameter_descriptor = ParameterDescriptor(description="Publish rate (Hz)")
        self.declare_parameter("rate", 1.0, parameter_descriptor)
        self.rate = self.get_parameter("rate").get_parameter_value().double_value

        self.publisher_ = self.create_publisher(
            TFMessage, "/tf2_msgs_msg_TFMessage", 10
        )
        timer_period = 1.0 / self.rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0.0
        self.y = 0.25
        self.z = 0.75
        self.yaw_deg = 0.0

    def timer_callback(self):
        msg = TFMessage()

        size = 50
        for i in range(size):
            x = self.x + 0.1 * i / size
            y = self.y + 0.1 * i / size
            z = self.z + 0.1 * i / size
            yaw_deg = self.yaw_deg + 0.1 * i / size

            yaw_rad = math.radians(yaw_deg)
            q = euler.euler2quat(0.0, 0.0, yaw_rad, 'ryxz')

            transform = Transform()
            transform.translation.x = x
            transform.translation.y = y
            transform.translation.z = z
            transform.rotation.x = q[1]
            transform.rotation.y = q[2]
            transform.rotation.z = q[3]
            transform.rotation.w = q[0]

            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            # transform_stamped.header.frame_id = "iris"
            transform_stamped.transform = transform

            msg.transforms.append(transform_stamped)

        self.x += 1.0
        self.y += 1.0
        self.z += 1.0
        self.yaw_deg += 1.0

        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: {}".format(msg))


def main(args=None):
    rclpy.init(args=args)

    tf_message_publisher = TFMessagePublisher(rate=1)

    rclpy.spin(tf_message_publisher)

    # Destroy the node prior to exit
    tf_message_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
