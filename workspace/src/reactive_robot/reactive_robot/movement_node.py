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
        angle_deg = msg.x
        distance = msg.y
        stop = msg.z

        twist = Twist()

        # --- 0. STOP COMMAND ---
        if stop == 1.0:
            self.vel_pub.publish(twist)
            self.get_logger().info('Mission Complete. Stopping.')
            return

        # --- 1. TURN ---
        # Normalize the angle: Turning 270 degrees left is the same as turning 90 degrees right.
        if angle_deg > 180:
            turn_angle_deg = 360.0 - angle_deg
            angular_speed = -0.5  # Spin right (negative)
        else:
            turn_angle_deg = angle_deg
            angular_speed = 0.5   # Spin left (positive)
            
        self.get_logger().info(f'MUSCLES: Starting movement! Turn: {turn_angle_deg}deg, Drive: {distance}m')

        if turn_angle_deg > 0:
            twist.angular.z = angular_speed
            self.vel_pub.publish(twist)
            
            
            turn_time = math.radians(turn_angle_deg) / abs(angular_speed)
            time.sleep(turn_time)

        # --- 2. MOVE FORWARD ---
        if distance > 0:
            # Stop turning, start moving forward
            twist.angular.z = 0.0
            twist.linear.x = 0.2  # Set linear speed to 0.2 m/s
            self.vel_pub.publish(twist)
            
            move_time = distance / 0.2
            time.sleep(move_time)

        # --- 3. STOP AND REPORT DONE ---
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_pub.publish(twist)
        
        # Send an empty Vector3 message back to the brain to say "I'm ready for the next move"
        done_msg = Vector3()
        self.done_pub.publish(done_msg)
        self.get_logger().info(f'Finished moving: turned {turn_angle_deg}deg, drove {distance}m.')

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