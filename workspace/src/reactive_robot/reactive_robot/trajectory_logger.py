import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import csv
import time

class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')
        self.sub = self.create_subscription(
            PoseArray, '/world/reactive_test/dynamic_pose/info', self.callback, 10)
        self.file = open('trajectory.csv', 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time', 'x', 'y'])
        self.start = None
        self.get_logger().info('Trajectory logger started')

    def callback(self, msg):
        now = time.time()
        if self.start is None:
            self.start = now
        if msg.poses:
            x = msg.poses[0].position.x
            y = msg.poses[0].position.y
            self.writer.writerow([now - self.start, x, y])
            self.file.flush()
            self.get_logger().info(f'x={x:.2f} y={y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TrajectoryLogger())

if __name__ == '__main__':
    main()