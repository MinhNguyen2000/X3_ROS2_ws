import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class MapOdomStaticTF(Node):
    """Publishes the fixed map -> <agent_name>_odom transform (the odom origin placed in the lab map)."""

    def __init__(self):
        super().__init__('map_odom_static_tf')

        self.declare_parameter('agent_name', 'agent0')
        self.declare_parameter('x', 0.61)        # 2 ft
        self.declare_parameter('y', 0.61)        # 2 ft
        self.declare_parameter('yaw', 0.0)       # rad

        self.agent_name = self.get_parameter('agent_name').value
        self.odom_frame = self.agent_name + '_odom'
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        yaw = float(self.get_parameter('yaw').value)

        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf(x, y, yaw)

        self.get_logger().info(
            f"Publishing static TF 'map' -> '{self.odom_frame}': x={x:.3f} m, y={y:.3f} m, yaw={yaw:.3f} rad")

    def publish_static_tf(self, x: float, y: float, yaw: float):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = self.odom_frame

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)

        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomStaticTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
