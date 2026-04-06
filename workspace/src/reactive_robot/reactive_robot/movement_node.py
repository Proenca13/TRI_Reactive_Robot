import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import math
import time
import threading

class MovementNode(Node):

    def __init__(self):
        super().__init__('movement_node')

        self.cmd_sub = self.create_subscription(
            Vector3, '/robot_command', self.command_callback, 10)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.done_pub = self.create_publisher(Vector3, '/done', 10)

        self._busy = False
        self.get_logger().info('Movement node started')

    def command_callback(self, msg: Vector3):
        if self._busy:
            self.get_logger().warn('Received command while busy — ignoring')
            return

        angle_deg = msg.x
        distance  = msg.y
        stop      = msg.z == 1.0

        if stop:
            self._publish_vel(0.0, 0.0)
            self.get_logger().info('Mission complete. Stopped.')
            return

        self._busy = True
        t = threading.Thread(
            target=self._execute_move,
            args=(angle_deg, distance),
            daemon=True
        )
        t.start()

    def _execute_move(self, angle_deg: float, distance: float):
        """Runs in a background thread. Blocks are fine here."""
        try:
            # ---- 1. TURN ----
            if angle_deg > 180:
                turn_deg   = 360.0 - angle_deg
                angular_sp = -0.5           # clockwise
            else:
                turn_deg   = angle_deg
                angular_sp =  0.5           # counter-clockwise

            if turn_deg > 1.0:
                self._publish_vel(0.0, angular_sp)
                time.sleep(math.radians(turn_deg) / abs(angular_sp))
                self._publish_vel(0.0, 0.0)
                time.sleep(0.05)

            # ---- 2. MOVE FORWARD ----
            LINEAR_SPEED = 0.2
            if distance > 0.001:
                self._publish_vel(LINEAR_SPEED, 0.0)
                time.sleep(distance / LINEAR_SPEED)
                self._publish_vel(0.0, 0.0)
                time.sleep(0.05)

        finally:
            self._busy = False
            # Tell the brain we're done
            self.done_pub.publish(Vector3())
            self.get_logger().info(
                f'Finished: turned {angle_deg:.0f}°, drove {distance:.2f} m')

    def _publish_vel(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
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