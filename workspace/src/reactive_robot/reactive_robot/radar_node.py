import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import math

class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')
        
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.listener_callback, 10)
            
        self.publisher_ = self.create_publisher(Float32MultiArray, '/processed_scan', 10)
            
        # REMOVED the self.done_sub and self.can_publish variable
        
        self.get_logger().info('Radar node started: Continuous Mode')
    
    # REMOVED the done_callback function entirely

    def listener_callback(self, msg: LaserScan):
        
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
        
        # Tip: You might want to comment out this logger if it prints too fast in continuous mode!
        self.get_logger().info(f'Bins are: {bins}') 
        
        self.publisher_.publish(processed_data)


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