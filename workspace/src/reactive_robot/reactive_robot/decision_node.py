import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
import math
import time
import numpy as np

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

        # --- Priority 3: wall nearby -> follow (left hand Maze Strategy) ---
        if self._wall_nearby(d):
            self.get_logger().info('Wall nearby - Following')
            self._wall_follow(d)
            return

        # --- Priority 4: no wall -> random movement or try to find wall ---
        self.get_logger().info('Wandering to find wall')
        self._wander(d)

    # -----------------------------------------------------------------------
    # Predicates
    # -----------------------------------------------------------------------

    def _wall_nearby(self, d):
        return min(d) < 0.8

    def _inside_circle(self, d):
        xs = []
        ys = []
        open_rays = 0

        # 1. Filter rays and count open space
        for i, dist in enumerate(d):
            if dist > 7.0:
                open_rays += 1
            else:
                angle = self._idx_to_rad(i)
                xs.append(dist * math.cos(angle))
                ys.append(dist * math.sin(angle))

        # INCREASED SAFEGUARD: The opening in the '5' is large.
        # We allow up to 10 rays to shoot into infinite space now.
        if open_rays > 10:
            return False

        if len(xs) < 15:
            return False

        x = np.array(xs)
        y = np.array(ys)

        # ---------------------------------------------------------
        # PASS 1: Rough Fit (Includes flat walls / noise)
        # ---------------------------------------------------------
        A_mat1 = np.c_[x, y, np.ones(len(x))]
        b_vec1 = x ** 2 + y ** 2
        try:
            c1, _, _, _ = np.linalg.lstsq(A_mat1, b_vec1, rcond=None)
            cx1, cy1 = c1[0] / 2.0, c1[1] / 2.0
            r_sq1 = cx1 ** 2 + cy1 ** 2 + c1[2]
            if r_sq1 < 0: return False
            r1 = np.sqrt(r_sq1)
        except np.linalg.LinAlgError:
            return False

        # Calculate how far each point is from the ROUGH circle
        dist1 = np.sqrt((x - cx1) ** 2 + (y - cy1) ** 2)
        residuals1 = np.abs(dist1 - r1)

        # ---------------------------------------------------------
        # PASS 2: Clean Fit (Discard flat walls)
        # ---------------------------------------------------------
        # Keep only points that are within 0.6 meters of our rough circle guess.
        # This effectively filters out the straight tail of the '5'.
        mask = residuals1 < 0.6
        x_clean = x[mask]
        y_clean = y[mask]

        # If we threw away too many points, it wasn't a circle to begin with
        if len(x_clean) < 10:
            return False

        # Recalculate using ONLY the clean, curved points
        A_mat2 = np.c_[x_clean, y_clean, np.ones(len(x_clean))]
        b_vec2 = x_clean ** 2 + y_clean ** 2
        try:
            c2, _, _, _ = np.linalg.lstsq(A_mat2, b_vec2, rcond=None)
            cx, cy = c2[0] / 2.0, c2[1] / 2.0
            r_sq2 = cx ** 2 + cy ** 2 + c2[2]
            if r_sq2 < 0: return False
            radius = np.sqrt(r_sq2)
        except np.linalg.LinAlgError:
            return False

        # ---------------------------------------------------------
        # FINAL CHECKS
        # ---------------------------------------------------------
        # Calculate error based on the newly refined circle
        dist2 = np.sqrt((x_clean - cx) ** 2 + (y_clean - cy) ** 2)
        residuals2 = np.abs(dist2 - radius)
        mean_error = np.mean(residuals2)

        # Because we filtered out the bad points, we can be much stricter with the error
        MAX_ERROR_THRESHOLD = 0.15
        MIN_EXPECTED_RADIUS = 0.8
        MAX_EXPECTED_RADIUS = 4.0

        is_circular = mean_error < MAX_ERROR_THRESHOLD
        is_correct_size = MIN_EXPECTED_RADIUS < radius < MAX_EXPECTED_RADIUS

        dist_to_center = math.hypot(cx, cy)
        is_physically_inside = dist_to_center < (radius * 0.9)

        return is_circular and is_correct_size and is_physically_inside
    
    def _get_exit_idx(self, d):
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
            if dist < 4.5:
                angle = self._idx_to_rad(i)
                xs.append(dist * math.cos(angle))
                ys.append(dist * math.sin(angle))
        
        if not xs:
            return 0.0, 0.0
            
        cx = (min(xs) + max(xs)) / 2.0
        cy = (min(ys) + max(ys)) / 2.0
        return cx, cy

    def _is_centered(self, d):
        cx, cy = self._get_room_center(d)
        dist_to_center = math.hypot(cx, cy)
        
        max_val = max(d)
        door_indices = [i for i, val in enumerate(d) if val == max_val]
        sum_sin = sum(math.sin(self._idx_to_rad(i)) for i in door_indices)
        sum_cos = sum(math.cos(self._idx_to_rad(i)) for i in door_indices)
        angle_to_exit = math.atan2(sum_sin, sum_cos)
        
        # We are done if we are physically in the center and facing the door
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
        
        # Drive to the geographical center first
        if dist_to_center > 0.25:
            angle_to_center = math.atan2(cy, cx)
            # Steer towards the geometric center
            angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, angle_to_center * 2.0))
            # Drive forward, slowing down if a sharp turn is needed
            linear_x = 0.2 if abs(angle_to_center) < 0.5 else 0.05
            self._send(angular_z, linear_x)
            return
            
        # Once perfectly centered geographically, spin in place to face the door
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
        # FRONT COLLISION OVERRIDE (Alignment)
        front_dists = [d[0], d[1], d[35]] 
        
        if min(front_dists) < 0.7:
            self._send(-1.0, 0.05) 
            return

        # We ignore ray 7 so that turning right 
        # doesn't cause the front-left sensor to clip the corner and cancel the jump.
        left_gap_dists = d[8:11] 
        
        if min(left_gap_dists) > 1.5:
            
            # Checks the right side to see if we are outside or in a corridor
            right_dists = d[24:30]
            right_is_open = min(right_dists) > 2.0
            
            if right_is_open:
                # TOP OPENING of the 5: Detach and jump.
                # Steer right a bit harder (-0.25) to safely clear the corner
                self._send(-0.25, FORWARD_SPEED) 
            else:
                # BOTTOM CIRCLE: Dive in.
                self._send(0.8, 0.15) 
            return

     
        left_dists = d[7:12]
        wall_dist = min(left_dists)
        
        # Proportional (P) - Distance Error
        error = wall_dist - TARGET_WALL_DISTANCE 
        
        # Derivative (D) - Heading Error (Ray 7 is fwd-left, Ray 11 is back-left)
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
        #COMPLETELY ISOLATED
        if min(d) >= 7.9:
            # We add a small constant (+0.2) to the sine wave.
            # This makes the robot turn slightly more left than right on average.
            # Instead of a straight line, it drives in a massive sweeping loop,
            # guaranteeing it will eventually turn around and sweep the whole map.
            angular_z = 0.8 * math.sin(time.time()) + 0.2
            self._send(angular_z, FORWARD_SPEED)
            
        # OBJECT DETECTED IN THE DISTANCE
        else:
            # As soon as ANY sensor detects something closer than 7.9m,
            # it moves in that direction
            min_idx = d.index(min(d))
            angle_error = self._idx_to_rad(min_idx)
            
            #Steer gently towards the closest object to find a wall
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