#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped

import copy

# The purpose of this file is to subscribe to the odom topic of the Gazebo
# plugin OdometryPublisher, which always have an odom-base_footprint offset
# when the agent has a spawn offset. This nodes correct the odom-base_footprint
# offset for navigation purposes

class OdomOffset(Node):
    def __init__(self):
        super().__init__("odom_offset_node")

        self.declare_parameter("agent_name", "agent0")
        self.declare_parameter("x_offset", 0.0)
        self.declare_parameter("y_offset", 0.0)

        agent_name = self.get_parameter("agent_name").get_parameter_value().string_value
        self.x_offset = self.get_parameter("x_offset").get_parameter_value().double_value
        self.y_offset = self.get_parameter("y_offset").get_parameter_value().double_value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.header_frame = f'{agent_name}_odom'
        self.child_frame = f'{agent_name}_base_footprint'

        # diff_drive_controller's default odom topic, relative to its own namespace
        self.sub = self.create_subscription(
            Odometry, f"/{agent_name}/odom_OP", self.odom_callback, 10
        )
        self.pub = self.create_publisher(Odometry, f"/{agent_name}/odom", 10)

        self.get_logger().info("odom_offset_node started!!")
        self.get_logger().info(f'x_offset={self.x_offset}, y_offset={self.y_offset}')


    def odom_callback(self, msg: Odometry):
        out = copy.deepcopy(msg)
        out.pose.pose.position.x = msg.pose.pose.position.x - self.x_offset
        out.pose.pose.position.y = msg.pose.pose.position.y - self.y_offset
        self.pub.publish(out)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.header_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = out.pose.pose.position.x
        t.transform.translation.y = out.pose.pose.position.y
        t.transform.translation.z = out.pose.pose.position.z
        t.transform.rotation = out.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomOffset()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()     
    rclpy.shutdown()

if __name__ == "__main__":
    main()