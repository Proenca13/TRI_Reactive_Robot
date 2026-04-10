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

        self.get_logger().info('Decision node started (Strict Circularity Mode)')

    def scan_callback(self, msg: Float32MultiArray):
        if self.done:
            return
            
        d = list(msg.data)
        self.step(d)

    def _analyze_room(self, d):
        """
        Instantly processes all LiDAR rays to find a best-fit circle.
        Uses Percentile Variance to strictly reject squares/polygons.
        """
        xs, ys = [], []
        for i, dist in enumerate(d):
            if dist < 7.5:  # Ignore infinite rays (the open doorway)
                angle = self._idx_to_rad(i)
                xs.append(dist * math.cos(angle))
                ys.append(dist * math.sin(angle))

        if len(xs) < 22: # Fast fail if we aren't highly enclosed
            return False, 0.0, 0.0, 0.0

        x = np.array(xs)
        y = np.array(ys)

        # --- PASS 1: Rough Circle Fit ---
        A = np.c_[x, y, np.ones(len(x))]
        b = x**2 + y**2
        try:
            c, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            cx1, cy1 = c[0]/2.0, c[1]/2.0
            r_sq1 = cx1**2 + cy1**2 + c[2]
            if r_sq1 < 0: return False, 0.0, 0.0, 0.0
            r1 = np.sqrt(r_sq1)
        except np.linalg.LinAlgError:
            return False, 0.0, 0.0, 0.0

        # =================================================================
        # THE ABSOLUTE POLYGON KILLER 
        # =================================================================
        # Calculate how far every laser hit is from our fitted center
        distances = np.sqrt((x - cx1)**2 + (y - cy1)**2)
        
        # We use percentiles (10% and 90%) to ignore messy rays hitting the doorframe.
        # This isolates the true flat walls (p10) and the deep corners (p90).
        p90 = np.percentile(distances, 90)
        p10 = np.percentile(distances, 10)
        
        # Calculate the percentage of variation. 
        # In a circle, this is ~0%. In a square, this is ~41%.
        variation = (p90 - p10) / r1
        
        # If the radius varies by more than 12%, it has corners! REJECT IT.
        if variation > 0.12: 
            return False, 0.0, 0.0, 0.0
        # =================================================================

        # --- PASS 2: Clean Outliers ---
        # Tighter margin: must be within 0.2m of the circle
        residuals = np.abs(distances - r1)
        mask = residuals < 0.2 
        x_clean = x[mask]
        y_clean = y[mask]

        if len(x_clean) < 22:
            return False, 0.0, 0.0, 0.0

        # --- PASS 3: Final Precision Fit ---
        A2 = np.c_[x_clean, y_clean, np.ones(len(x_clean))]
        b2 = x_clean**2 + y_clean**2
        try:
            c2, _, _, _ = np.linalg.lstsq(A2, b2, rcond=None)
            cx, cy = c2[0]/2.0, c2[1]/2.0
            r_sq2 = cx**2 + cy**2 + c2[2]
            if r_sq2 < 0: return False, 0.0, 0.0, 0.0
            r2 = np.sqrt(r_sq2)
        except np.linalg.LinAlgError:
            return False, 0.0, 0.0, 0.0

        # Check if the robot is physically sitting inside this mathematical circle
        dist_to_center = math.hypot(cx, cy)
        is_inside = dist_to_center < (r2 * 0.9)

        if is_inside:
            return True, cx, cy, r2
            
        return False, 0.0, 0.0, 0.0

    def step(self, d):
        # Run the regression analysis once per frame
        is_circle, cx, cy, radius = self._analyze_room(d)

        # --- Priority 1: centered inside circle and heading out -> stop ---
        if is_circle and self._is_centered(d, cx, cy):
            self.get_logger().info('Centered and heading out! Mission complete.')
            self._send(0.0, 0.0, stop=True)
            self.done = True
            return

        # --- Priority 2: inside circle -> center and orient ---
        if is_circle:
            self.get_logger().info('Inside circle - Centering via Regression')
            self._center(d, cx, cy)
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
    
    def _is_centered(self, d, cx, cy):
        dist_to_center = math.hypot(cx, cy)
        
        # Find the exact middle of the doorway
        max_val = max(d)
        door_indices = [i for i, val in enumerate(d) if val >= max_val - 0.5]
        if not door_indices:
            return False
            
        sum_sin = sum(math.sin(self._idx_to_rad(i)) for i in door_indices)
        sum_cos = sum(math.cos(self._idx_to_rad(i)) for i in door_indices)
        angle_to_exit = math.atan2(sum_sin, sum_cos)
        
        # We are done if we are physically in the center and facing the door
        return dist_to_center < 0.15 and abs(angle_to_exit) < 0.2 

    def _center(self, d, cx, cy):
        dist_to_center = math.hypot(cx, cy)
        
        # Calculate where the door is
        max_val = max(d)
        door_indices = [i for i, val in enumerate(d) if val >= max_val - 0.5]
        if door_indices:
            sum_sin = sum(math.sin(self._idx_to_rad(i)) for i in door_indices)
            sum_cos = sum(math.cos(self._idx_to_rad(i)) for i in door_indices)
            angle_to_exit = math.atan2(sum_sin, sum_cos)
        else:
            angle_to_exit = 0.0
        
        # 1. Drive to the geographical center first
        if dist_to_center > 0.15:
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
        # FRONT COLLISION OVERRIDE (Alignment)
        front_dists = [d[0], d[1], d[35]] 
        
        if min(front_dists) < 0.7:
            self._send(-1.0, 0.05) 
            return

        # EARLY GAP DETECTION 
        left_gap_dists = d[8:11] 
        
        if min(left_gap_dists) > 1.5:
            right_dists = d[24:30]
            right_is_open = min(right_dists) > 2.0
            
            if right_is_open:
                # TOP OPENING: Detach and jump
                self._send(-0.25, FORWARD_SPEED) 
            else:
                # BOTTOM CIRCLE: Dive in
                self._send(0.8, 0.15) 
            return

        # P/D CONTROLLER
        left_dists = d[7:12]
        wall_dist = min(left_dists)
        
        error = wall_dist - TARGET_WALL_DISTANCE 
        heading_error = d[7] - d[11]
        
        error = max(-0.5, min(0.5, error))
        heading_error = max(-0.5, min(0.5, heading_error))
        
        correction = (1.5 * error) + (0.8 * heading_error)
        angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, correction))
        adaptive_speed = FORWARD_SPEED if abs(angular_z) < 0.3 else 0.1

        self._send(angular_z, adaptive_speed)

    def _wander(self, d):
        # COMPLETELY ISOLATED
        if min(d) >= 7.9:
            angular_z = 0.8 * math.sin(time.time()) + 0.2
            self._send(angular_z, FORWARD_SPEED)
            
        # OBJECT DETECTED IN THE DISTANCE
        else:
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