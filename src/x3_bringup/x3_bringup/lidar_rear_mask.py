#!/usr/bin/env python
"""
lidar_rear_mask_node.py

Masks a fixed angular window (default: directly behind the robot) out of an
incoming sensor_msgs/LaserScan before republishing it. Intended to sit
between the RPLiDAR driver and rf2o. To prevent LiDAR noise coming from hardware
behind the physical LiDAR
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle (radians) into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class LidarRearMaskNode(Node):
    def __init__(self):
        super().__init__('lidar_rear_mask_node')

        # ===== Parameters =====
        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_filtered')
        # Angular center of the masked window, in the scan's own frame
        # (radians). pi == directly behind the sensor's local +X axis.
        self.declare_parameter('mask_center_angle', math.pi)
        # Half-width of the masked window, in radians, on each side of
        # mask_center_angle. Total masked arc = 2 * mask_half_width.
        self.declare_parameter('mask_half_width', 0.35)
        # 'zero' -> range = 0.0 (rf2o drops these points outright)
        # 'range_max' -> range = scan.range_max (finite, "no obstacle")
        self.declare_parameter('mask_value', 'zero')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.mask_center = wrap_to_pi(
            self.get_parameter('mask_center_angle').value)
        self.mask_half_width = float(
            self.get_parameter('mask_half_width').value)
        self.mask_value_mode = self.get_parameter('mask_value').value

        if self.mask_value_mode not in ('zero', 'range_max'):
            self.get_logger().warn(
                f"Unknown mask_value '{self.mask_value_mode}', "
                f"defaulting to 'zero'."
            )
            self.mask_value_mode = 'zero'

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(LaserScan, self.input_topic, self.scan_callback, qos)
        self.pub = self.create_publisher(LaserScan, self.output_topic, qos)

        self.get_logger().info(
            f"Masking [{self.mask_center - self.mask_half_width:.3f}, "
            f"{self.mask_center + self.mask_half_width:.3f}] rad "
            f"(centered on {self.mask_center:.3f} rad) "
            f"from '{self.input_topic}' -> '{self.output_topic}', "
            f"mode='{self.mask_value_mode}'"
        )

    def scan_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            self.pub.publish(msg)
            return

        mask_value = 0.0 if self.mask_value_mode == 'zero' else msg.range_max
        has_intensities = len(msg.intensities) == n

        ranges = list(msg.ranges)
        intensities = list(msg.intensities) if has_intensities else None

        for i in range(n):
            angle = wrap_to_pi(msg.angle_min + i * msg.angle_increment)
            diff = abs(wrap_to_pi(angle - self.mask_center))
            if diff <= self.mask_half_width:
                ranges[i] = mask_value
                if has_intensities:
                    intensities[i] = 0.0

        msg.ranges = ranges
        if has_intensities:
            msg.intensities = intensities

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarRearMaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()