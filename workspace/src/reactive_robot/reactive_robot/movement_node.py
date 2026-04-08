import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import math
import time

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')

        # Listen to the Brain
        self.cmd_sub = self.create_subscription(
            Vector3, '/robot_command', self.command_callback, 10)

        # Talk to the Wheels
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Tell the Brain we finished moving
        self.done_pub = self.create_publisher(Vector3, '/done', 10)

        self.get_logger().info('Movement node (Muscles) started')

    def command_callback(self, msg: Vector3):
       

        twist = Twist()

        # --- 0. STOP COMMAND ---
        if msg.z == 1.0:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.vel_pub.publish(twist)
            self.get_logger().info('Mission Complete. Stopping.')
            return

        # --- 1. CONTINUOUS MOVEMENT ---
        twist.linear.x = float(msg.y)
        twist.angular.z = float(msg.x)
        
        # Publish the speeds immediately
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