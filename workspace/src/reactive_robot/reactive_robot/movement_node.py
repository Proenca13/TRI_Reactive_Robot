import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3


class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')

        # Subscribe to the decision node's continuous command
        self.cmd_sub = self.create_subscription(Vector3, '/robot_command', self.cmd_callback, 10)

        # Publish directly to the robot's wheels
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Memory-less Movement node started! Awaiting speeds.')

    def cmd_callback(self, msg: Vector3):

        twist = Twist()

        twist.linear.x = float(msg.x)
        twist.angular.z = float(msg.y)

        # Instantly publish to the wheels
        self.vel_pub.publish(twist)


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