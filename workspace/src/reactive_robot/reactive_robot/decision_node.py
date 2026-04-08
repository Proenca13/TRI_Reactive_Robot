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
CENTERED_TOLERANCE = 0.05

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
        # Count how many rays hit a wall (detects if we are surrounded)
        rays_hitting_wall = sum(1 for dist in d if dist < 4.5)
        
        # Count how many rays shoot into infinite open space
        open_rays = sum(1 for dist in d if dist > 7.0)
        
        # If we have more than 10 rays (~100 degrees) of open space, 
        # we are in the wide-open square bay, NOT the enclosed circle!
        return (rays_hitting_wall >= 19) and (open_rays < 5)
    
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

    def _wall_follow(self, d):
        # 1. FRONT COLLISION OVERRIDE (Alignment)
        front_dists = [d[0], d[1], d[35]] 
        
        if min(front_dists) < 0.7:
            self._send(-1.0, 0.05)  # Spin right, creep forward slowly 
            return

        # 2. EARLY GAP DETECTION (The "Invisible Wall" Override)
        # STRICTLY sideways: We ignore ray 7 so that turning right 
        # doesn't cause the front-left sensor to clip the corner and cancel the jump!
        left_gap_dists = d[8:11] 
        
        if min(left_gap_dists) > 1.5:
            
            # Check the right side to see if we are outside or in a corridor
            right_dists = d[24:30]
            right_is_open = min(right_dists) > 2.0
            
            if right_is_open:
                # TOP OPENING: Detach and jump!
                # Steer right a bit harder (-0.25) to safely clear the corner
                self._send(-0.25, FORWARD_SPEED) 
            else:
                # BOTTOM CIRCLE: Dive in!
                self._send(0.8, 0.15) 
            return

        # ==========================================
        # 3. P/D CONTROLLER (STANDARD WALL FOLLOWING)
        # ==========================================
        # NARROWED VISION: Only look at rays 7 to 11 so the robot doesn't
        # try to "chase" the curving wall in front of it into the bay.
        left_dists = d[7:12]
        wall_dist = min(left_dists)
        
        # 1. Proportional (P) - Distance Error
        error = wall_dist - TARGET_WALL_DISTANCE 
        
        # 2. Derivative (D) - Heading Error (Ray 7 is fwd-left, Ray 11 is back-left)
        heading_error = d[7] - d[11]
        
        # Cap errors to prevent wild swings from sensor noise
        error = max(-0.5, min(0.5, error))
        heading_error = max(-0.5, min(0.5, heading_error))
        
        # Combine distance correction and heading correction
        correction = (1.5 * error) + (0.8 * heading_error)
        
        angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, correction))

        # Adaptive speed: drive faster if driving straight, slower if turning hard
        adaptive_speed = FORWARD_SPEED if abs(angular_z) < 0.3 else 0.1

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