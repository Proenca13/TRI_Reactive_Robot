import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ReactiveNode(Node):
    def __init__(self):
        super().__init__('reactive_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.last_ranges = None
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Reactive robot node started')

    def scan_callback(self, msg: LaserScan):
        self.last_ranges = msg.ranges

    def min_range_in_window(self, ranges, start_deg, end_deg):
        if ranges is None or len(ranges) == 0:
            return float('inf')

        total = len(ranges)
        center = total // 2

        start_idx = max(0, center + start_deg)
        end_idx = min(total - 1, center + end_deg)

        values = []
        for i in range(start_idx, end_idx + 1):
            value = ranges[i]
            if math.isfinite(value):
                values.append(value)

        if not values:
            return float('inf')

        return min(values)

    def control_loop(self):
        if self.last_ranges is None:
            return

        front = self.min_range_in_window(self.last_ranges, -10, 10)
        left = self.min_range_in_window(self.last_ranges, 20, 70)
        right = self.min_range_in_window(self.last_ranges, -70, -20)

        msg = Twist()

        if front < 0.7:
            msg.linear.x = 0.0
            msg.angular.z = 0.8
        else:
            msg.linear.x = 0.3
            msg.angular.z = 0.0

            if left < 0.5:
                msg.angular.z = -0.3
            elif right < 0.5:
                msg.angular.z = 0.3

        self.cmd_pub.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()