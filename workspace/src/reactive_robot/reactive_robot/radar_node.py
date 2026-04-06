import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
import math

class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')
        
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.listener_callback, 10)
            
        self.publisher_ = self.create_publisher(Float32MultiArray, '/processed_scan', 10)
        self.done_sub = self.create_subscription(
            Vector3, '/done', self.done_callback, 10)
            
        self.can_publish = True 
        self.get_logger().info('Radar node started: Turn-Based Mode')
    
    def done_callback(self, msg: Vector3):
        # The wheels finished moving! We are allowed to take a fresh picture.
        self.get_logger().info('RADAR: Received /done signal! Waking up for next picture.')
        self.can_publish = True

    def listener_callback(self, msg: LaserScan):
        # If the robot is currently moving, IGNORE the blurry laser scans!
        if not self.can_publish:
            return
        
        bins = [float('inf')] * 36

        for i, distance in enumerate(msg.ranges):
            if math.isnan(distance) or not math.isfinite(distance) or distance < 0.20:
                continue

            angle_rad = msg.angle_min + (i * msg.angle_increment)
            angle_deg = math.degrees(angle_rad) % 360.0

            bin_index = int(round(angle_deg / 10.0)) % 36

            if distance < bins[bin_index]:
                bins[bin_index] = float(distance)

        max_range = float(msg.range_max) if math.isfinite(msg.range_max) else 10.0
        bins = [b if b != float('inf') else max_range for b in bins]

        processed_data = Float32MultiArray()
        processed_data.data = bins
        self.publisher_.publish(processed_data)

        self.get_logger().info('RADAR: SNAP! Picture sent to Brain. Going to sleep.')
        self.can_publish = False


def main(args=None):
    rclpy.init(args=args)
    node = RadarNode()
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