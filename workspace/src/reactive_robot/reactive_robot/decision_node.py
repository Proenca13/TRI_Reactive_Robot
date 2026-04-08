import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
import math

# ---------------------------------------------------------------------------
# Tuning parameters
# ---------------------------------------------------------------------------

TARGET_WALL_DISTANCE = 0.5  # Desired distance to the wall (meters)
FORWARD_SPEED = 0.2         # Base forward speed (m/s)
MAX_ANGULAR = 1.0           # Maximum turning speed (rad/s)
CENTERED_TOLERANCE = 0.1

FRONT_IDX = 0
LEFT_IDX = 9
BACK_IDX = 18
RIGHT_IDX = 27

# ---------------------------------------------------------------------------

class DecisionNode(Node):

    def __init__(self):
        super().__init__('decision_node')

        self.scan_sub = self.create_subscription(
            Float32MultiArray, '/processed_scan', self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(Vector3, '/robot_command', 10)

        self.done = False

        # PI — only allowed memory
        self.pi_integral = 0.0

        self.get_logger().info('Decision node started (Continuous Reactive Mode)')

    def scan_callback(self, msg: Float32MultiArray):
        if self.done:
            return
            
        d = list(msg.data)
        self.step(d)

    def step(self, d):
        # --- Priority 1: centered inside circle and heading out -> stop ---
        if self._inside_circle(d) and self._is_centered(d):
            self.get_logger().info('Centered and heading out! Mission complete.')
            self._send(0.0, 0.0, stop=True)
            self.done = True
            return

        # --- Priority 2: inside circle -> center and orient ---
        if self._inside_circle(d):
            self.get_logger().info('Inside circle - Centering')
            self._center(d)
            return

        # --- Priority 3: wall nearby -> follow (Maze Strategy) ---
        # The left-hand rule will naturally guide us through the door!
        if self._wall_nearby(d):
            self.get_logger().info('Wall nearby - Following')
            self._wall_follow(d)
            return

        # --- Priority 4: no wall -> random movement ---
        self.get_logger().info('Wandering to find wall')
        self._wander(d)

    # -----------------------------------------------------------------------
    # Predicates
    # -----------------------------------------------------------------------

    def _wall_nearby(self, d):
        return min(d) < 0.8

    def _inside_circle(self, d):
        # BULLETPROOF CHECK: Instead of relying on the robot's rotation, 
        # we just count how many rays are hitting a wall.
        # If 24 or more rays hit a wall within 4.5 meters, we are fully enclosed!
        rays_hitting_wall = sum(1 for dist in d if dist < 4.5)
        return rays_hitting_wall >= 24

    # NOTE: You can completely DELETE the _circle_detected and _enter_circle functions!

    def _get_exit_idx(self, d):
        # Vector-average all the rays looking through the gap to find the TRUE center
        max_val = max(d)
        x = 0.0
        y = 0.0
        for i, val in enumerate(d):
            if val >= max_val - 0.2:
                angle = self._idx_to_rad(i)
                x += math.cos(angle)
                y += math.sin(angle)
                
        angle_rad = math.atan2(y, x)
        angle_deg = math.degrees(angle_rad)
        if angle_deg < 0:
            angle_deg += 360.0
            
        return int(round(angle_deg / 10.0)) % 36

    def _get_room_center(self, d):
        xs = []
        ys = []
        for i, dist in enumerate(d):
            if dist < 4.5:  # Ignore the door opening (the 8.0s)
                angle = self._idx_to_rad(i)
                # Convert polar (distance, angle) to cartesian (x, y)
                xs.append(dist * math.cos(angle))
                ys.append(dist * math.sin(angle))
        
        if not xs:
            return 0.0, 0.0
            
        # The Bounding Box Trick: Halfway between the extremes is the perfect center!
        cx = (min(xs) + max(xs)) / 2.0
        cy = (min(ys) + max(ys)) / 2.0
        return cx, cy

    def _is_centered(self, d):
        cx, cy = self._get_room_center(d)
        dist_to_center = math.hypot(cx, cy)
        
        # Find the exact middle of the doorway using circular mean
        max_val = max(d)
        door_indices = [i for i, val in enumerate(d) if val == max_val]
        sum_sin = sum(math.sin(self._idx_to_rad(i)) for i in door_indices)
        sum_cos = sum(math.cos(self._idx_to_rad(i)) for i in door_indices)
        angle_to_exit = math.atan2(sum_sin, sum_cos)
        
        # We are done if we are physically in the center AND facing the door
        return dist_to_center < 0.25 and abs(angle_to_exit) < 0.2

    def _center(self, d):
        cx, cy = self._get_room_center(d)
        dist_to_center = math.hypot(cx, cy)
        
        # Calculate where the door is
        max_val = max(d)
        door_indices = [i for i, val in enumerate(d) if val == max_val]
        sum_sin = sum(math.sin(self._idx_to_rad(i)) for i in door_indices)
        sum_cos = sum(math.cos(self._idx_to_rad(i)) for i in door_indices)
        angle_to_exit = math.atan2(sum_sin, sum_cos)
        
        # 1. Drive to the geographical center first
        if dist_to_center > 0.25:
            angle_to_center = math.atan2(cy, cx)
            # Steer towards the geometric center
            angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, angle_to_center * 2.0))
            # Drive forward, slowing down if a sharp turn is needed
            linear_x = 0.2 if abs(angle_to_center) < 0.5 else 0.05
            self._send(angular_z, linear_x)
            return
            
        # 2. Once perfectly centered geographically, spin in place to face the door
        angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, angle_to_exit * 1.5))
        self._send(angular_z, 0.0)

    # -----------------------------------------------------------------------
    # Helper Math
    # -----------------------------------------------------------------------

    def _idx_to_rad(self, idx):
        angle_deg = idx * 10.0
        if angle_deg > 180:
            angle_deg -= 360.0
        return math.radians(angle_deg)

    # -----------------------------------------------------------------------
    # Actions
    # -----------------------------------------------------------------------

    def _enter_circle(self, d):
        # Steer towards the true center of the gap
        exit_idx = self._get_exit_idx(d)
        angle_error = self._idx_to_rad(exit_idx)
        
        angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, angle_error * 1.5))
        # Drive slower while entering so we don't clip the walls!
        self._send(angular_z, FORWARD_SPEED * 0.5) 


    def _wall_follow(self, d):
        # 1. FRONT COLLISION OVERRIDE (Alignment)
        # Check a narrow cone directly in front of the robot (Bins 0-2 and 33-35)
        front_dists = d[0:3] + d[33:36]
        
        # If a wall is physically blocking our forward path, 
        # spin RIGHT to slide the wall onto our left shoulder!
        if min(front_dists) < 0.85:
            self._send(-1.0, 0.05)  # Spin right, creep forward slowly 
            self.pi_integral = 0.0  # Reset PI memory
            return

        # 2. LEFT WALL ORBIT (PI Controller)
        # The front is clear! Look at the left side (Bins 4 to 13)
        left_dists = d[4:14]
        wall_dist = min(left_dists)
        
        # If the wall drops away completely (e.g., an external corner)
        # gently turn left to wrap around the corner and find it again.
        if wall_dist > 1.5:
            self._send(0.8, 0.15) 
            self.pi_integral = 0.0
            return

        # P/I controller math (positive correction = turn left towards wall)
        error = wall_dist - TARGET_WALL_DISTANCE 
        self.pi_integral = max(-1.0, min(1.0, self.pi_integral + error))
        
        correction = (0.8 * error) + (0.1 * self.pi_integral)
        angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, correction))

        # Adaptive speed: drive faster if driving straight, slower if turning hard
        adaptive_speed = FORWARD_SPEED if abs(angular_z) < 0.5 else 0.1

        self._send(angular_z, adaptive_speed)

    def _wander(self, d):
        # Steer gently towards the closest object to find a wall
        min_idx = d.index(min(d))
        angle_error = self._idx_to_rad(min_idx)
        
        angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, angle_error * 0.5))
        self._send(angular_z, FORWARD_SPEED)

    # -----------------------------------------------------------------------

    def _send(self, angular_z: float, linear_x: float, stop: bool = False):
        msg = Vector3()
        msg.x = float(angular_z)
        msg.y = float(linear_x)
        msg.z = 1.0 if stop else 0.0
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
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