#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class OdomToVelRaw(Node):
    def __init__(self):
        super().__init__("odom_to_vel_raw_node")

        self.declare_parameter("agent_name", "agent0")
        agent_name = self.get_parameter("agent_name").get_parameter_value().string_value

        # diff_drive_controller's default odom topic, relative to its own namespace
        self.sub = self.create_subscription(
            Odometry, f"/{agent_name}/wheel_odom", self.odom_callback, 10
        )
        self.pub = self.create_publisher(TwistStamped, f"/{agent_name}/vel_raw", 10)

        self.get_logger().info("odom_to_vel_raw_node started!!")

    def odom_callback(self, msg: Odometry):
        out = TwistStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.child_frame_id
        out.twist = msg.twist.twist
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToVelRaw()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()