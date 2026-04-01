import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
import math

# ---------------------------------------------------------------------------
# Tuning parameters
# ---------------------------------------------------------------------------

TARGET_WALL_DISTANCE = 0.3      # metres — desired distance from wall
WANDER_WALL_THRESHOLD = 1.0     # start wall-follow when wall closer than this
STEP_DISTANCE = 0.1             # metres per movement step

# PI gains for wall-following lateral correction
KP = 1.2
KI = 0.15
PI_INTEGRAL_LIMIT = 1.0         # anti-windup clamp

# Circle detection: right-side distances must be stable AND close
CIRCLE_MIN_DIST = 0.15          # circle wall must be at least this close
CIRCLE_MAX_DIST = 0.8           # and no further than this
CIRCLE_VARIANCE_THRESHOLD = 0.01  # low variance = uniform curvature

# Sector index layout (36 readings, one per 10 degrees)
# Front = index 0 (0°), Right = index 27 (270°), Left = index 9 (90°)
FRONT_IDX = 0
LEFT_IDX = 9
BACK_IDX = 18
RIGHT_IDX = 27

# Right-side sector used for wall-following and circle detection
RIGHT_SECTOR_START = 24   # 240°
RIGHT_SECTOR_END   = 30   # 300°

# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------

STATE_WANDER = 'WANDER'
STATE_WALL_FOLLOW = 'WALL_FOLLOW'
STATE_CIRCLE_FOUND = 'CIRCLE_FOUND'
STATE_CENTER = 'CENTER'
STATE_DONE = 'DONE'


class DecisionNode(Node):

    def __init__(self):
        super().__init__('decision_node')

        self.scan_sub = self.create_subscription(
            Float32MultiArray, '/processed_scan', self.scan_callback, 10)

        self.done_sub = self.create_subscription(
            Vector3, '/done', self.done_callback, 10)

        # x = heading angle (degrees), y = distance (metres), z = stop flag
        self.cmd_pub = self.create_publisher(Vector3, '/robot_command', 10)

        self.state = STATE_WANDER
        self.waiting = False
        self.last_scan = None

        # PI state — only persistent data allowed by assignment rules
        # (assignment explicitly permits PID memory)
        self.pi_integral = 0.0
        self.pi_prev_error = 0.0

        self.get_logger().info('Decision node started — state: WANDER')

    # -----------------------------------------------------------------------
    # ROS callbacks
    # -----------------------------------------------------------------------

    def scan_callback(self, msg: Float32MultiArray):
        self.last_scan = list(msg.data)
        # Trigger first cycle from scan (before any /done arrives)
        if self.state == STATE_WANDER and not self.waiting:
            self.step()

    def done_callback(self, _msg: Vector3):
        self.waiting = False
        self.step()

    # -----------------------------------------------------------------------
    # Main decision loop
    # -----------------------------------------------------------------------

    def step(self):
        if self.last_scan is None or self.waiting:
            return

        d = self.last_scan   # shorthand: 36 distances, index = degrees / 10

        if self.state == STATE_WANDER:
            self._wander(d)
        elif self.state == STATE_WALL_FOLLOW:
            self._wall_follow(d)
        elif self.state == STATE_CIRCLE_FOUND:
            self._enter_circle(d)
        elif self.state == STATE_CENTER:
            self._center(d)
        # STATE_DONE: do nothing

    # -----------------------------------------------------------------------
    # WANDER — move toward the nearest wall
    # -----------------------------------------------------------------------

    def _wander(self, d):
        min_dist = min(d)
        min_idx = d.index(min_dist)

        if min_dist < WANDER_WALL_THRESHOLD:
            self.get_logger().info('Wall detected — switching to WALL_FOLLOW')
            self._reset_pi()
            self.state = STATE_WALL_FOLLOW
            self._wall_follow(d)
            return

        # Head toward closest obstacle
        angle = min_idx * 10
        self._send(angle, STEP_DISTANCE)

    # -----------------------------------------------------------------------
    # WALL_FOLLOW — keep the wall on the right at TARGET_WALL_DISTANCE
    #               using a PI controller; detect circle on the way
    # -----------------------------------------------------------------------

    def _wall_follow(self, d):
        if self._circle_detected(d):
            self.get_logger().info('Circular wall detected — switching to CIRCLE_FOUND')
            self.state = STATE_CIRCLE_FOUND
            self._enter_circle(d)
            return

        # Minimum distance on the right sector
        right_dists = d[RIGHT_SECTOR_START:RIGHT_SECTOR_END + 1]
        wall_dist = min(right_dists)

        # PI control
        error = wall_dist - TARGET_WALL_DISTANCE
        self.pi_integral = max(-PI_INTEGRAL_LIMIT,
                               min(PI_INTEGRAL_LIMIT,
                                   self.pi_integral + error))
        correction = KP * error + KI * self.pi_integral

        # correction > 0  → too far → steer right (toward wall)
        # correction < 0  → too close → steer left (away from wall)
        # We clamp the steering angle to ±60° from forward (0°)
        steer_deg = max(-60.0, min(60.0, correction * 30.0))

        # Forward with a lateral nudge: angle 0 = straight ahead
        # Positive steer_deg → turn right (clockwise → subtract from 360)
        if steer_deg >= 0:
            angle = int(360 - steer_deg) % 360   # e.g. 350° = slightly right
        else:
            angle = int(-steer_deg)               # e.g. 10° = slightly left

        self._send(angle, STEP_DISTANCE)

    # -----------------------------------------------------------------------
    # Circle detection — right-side readings are close AND low-variance
    # -----------------------------------------------------------------------

    def _circle_detected(self, d):
        right = d[RIGHT_SECTOR_START:RIGHT_SECTOR_END + 1]
        
        if not all(CIRCLE_MIN_DIST <= v <= CIRCLE_MAX_DIST for v in right):
            return False

        mean = sum(right) / len(right)
        variance = sum((v - mean) ** 2 for v in right) / len(right)

        return mean >= CIRCLE_MIN_DIST and variance < CIRCLE_VARIANCE_THRESHOLD

    # -----------------------------------------------------------------------
    # CIRCLE_FOUND — find the gap (opening of the "5") and drive through it
    # -----------------------------------------------------------------------

    def _enter_circle(self, d):
        # 3 out of 4 cardinals have walls close = we are inside the circle
        cardinals = [d[FRONT_IDX], d[RIGHT_IDX], d[BACK_IDX], d[LEFT_IDX]]
        walls = [v for v in cardinals if v < WANDER_WALL_THRESHOLD]
        if len(walls) >= 3:
            self.get_logger().info('Inside circle — switching to CENTER')
            self._reset_pi()
            self.state = STATE_CENTER
            self._center(d)
            return

        # Move toward the opening (maximum distance)
        max_idx = d.index(max(d))
        angle = max_idx * 10
        self.get_logger().info(f'Entering circle through gap at {angle}°')
        self._send(angle, STEP_DISTANCE)

    # -----------------------------------------------------------------------
    # CENTER — move to the geometric centre of the circle
    #          Stop criterion: the three "wall" cardinals are balanced
    #          and the robot faces the opening
    # -----------------------------------------------------------------------

    def _center(self, d):
        # Cardinal distances: front, right, back, left
        df = d[FRONT_IDX]
        dr = d[RIGHT_IDX]
        db = d[BACK_IDX]
        dl = d[LEFT_IDX]

        # Lateral and longitudinal imbalance
        lateral_err = dr - dl          # >0 → too far right → move left
        longitudinal_err = df - db          # >0 → too far forward → move back

        tolerance = 0.05   # metres — tighter tolerance with exact moves

        if abs(lateral_err) < tolerance and abs(longitudinal_err) < tolerance:
            # Centred — orient toward the opening (maximum distance)
            max_idx = d.index(max(d))
            exit_angle = max_idx * 10
            self.get_logger().info(
                f'Centred! Facing exit at {exit_angle}°. Mission complete.')
            self._send(exit_angle, 0.0, stop=True)
            self.state = STATE_DONE
            return

        # Move exactly half the imbalance — lands on centre in one step
        if abs(lateral_err) >= abs(longitudinal_err):
            angle = 270 if lateral_err > 0 else 90   # right or left
            move_dist = abs(lateral_err) / 2.0
        else:
            angle = 180 if longitudinal_err > 0 else 0   # back or forward
            move_dist = abs(longitudinal_err) / 2.0

        self._send(angle, move_dist)

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _reset_pi(self):
        self.pi_integral  = 0.0
        self.pi_prev_error = 0.0

    def _send(self, angle: float, distance: float, stop: bool = False):
        msg   = Vector3()
        msg.x = float(angle)
        msg.y = float(distance)
        msg.z = 1.0 if stop else 0.0
        self.cmd_pub.publish(msg)
        self.waiting = True
        self.get_logger().info(
            f'[{self.state}] cmd angle={angle:.0f}° dist={distance:.2f}m stop={stop}')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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