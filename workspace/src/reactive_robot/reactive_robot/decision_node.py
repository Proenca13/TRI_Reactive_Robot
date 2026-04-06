import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
import random

# ---------------------------------------------------------------------------
# Tuning parameters (only what cannot be derived mathematically)
# ---------------------------------------------------------------------------

TARGET_WALL_DISTANCE = 0.3
STEP_DISTANCE = 0.1
CENTERED_TOLERANCE = 0.05
JUMP_FACTOR = 3.0  # how abrupt the jump at the circle opening must be

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

        self.done_sub = self.create_subscription(
            Vector3, '/done', self.done_callback, 10)

        self.cmd_pub = self.create_publisher(Vector3, '/robot_command', 10)

        self.waiting = False
        self.last_scan = None
        self.done = False

        # PI — only allowed memory
        self.pi_integral = 0.0

        self.get_logger().info('Decision node started')

    def scan_callback(self, msg: Float32MultiArray):
        self.last_scan = list(msg.data)
        if not self.waiting:
            self.step()

    def done_callback(self, _msg: Vector3):
        self.waiting = False

    def step(self):
        if self.last_scan is None or self.waiting or self.done:
            return

        d = self.last_scan

        # --- Priority 1: centered inside circle and heading out -> stop ---
        if self._inside_circle(d) and self._is_centered(d):
            self.get_logger().info('Centered and heading out! Mission complete.')
            self._send(0, 0.0, stop=True)
            self.done = True
            return

        # --- Priority 2: inside circle -> center and orient ---
        if self._inside_circle(d):
            self.get_logger().info('Inside circle')
            self._center(d)
            return

        # --- Priority 3: circular wall detected -> enter ---
        if self._circle_detected(d):
            self.get_logger().info('Circle Detected')
            self._enter_circle(d)
            return

        # --- Priority 4: wall nearby -> follow ---
        if self._wall_nearby(d):
            self.get_logger().info('Wall nearby')
            self._wall_follow(d)
            return

        # --- Priority 5: no wall -> random movement ---
        self._wander(d)

    # -----------------------------------------------------------------------
    # Predicates
    # -----------------------------------------------------------------------

    def _find_pairs(self, d):
        max_idx = d.index(max(d))

        def near_opening(idx):
            diff = min(abs(idx - max_idx), 36 - abs(idx - max_idx))
            return diff <= 3

        pair1 = None
        for i in range(1, 18):
            idx_a = (max_idx + i) % 36
            idx_b = (max_idx + i + 18) % 36
            if not near_opening(idx_a) and not near_opening(idx_b):
                pair1 = (idx_a, idx_b)
                break

        if pair1 is None:
            return None, None

        pair2 = None
        for i in range(pair1[0] + 7, pair1[0] + 12):
            idx_a = i % 36
            idx_b = (i + 18) % 36
            if not near_opening(idx_a) and not near_opening(idx_b):
                pair2 = (idx_a, idx_b)
                break

        return pair1, pair2

    def _wall_nearby(self, d):
        # Wall is nearby if the minimum distance is strictly less than the mean
        mean = sum(d) / len(d)
        return min(d) < mean

    def _inside_circle(self, d):
        # Choose 4 cardinal directions with the maximum guaranteed in one of them
        # If 3 out of 4 are smaller than the maximum -> robot is inside the circle
        max_idx = d.index(max(d))
        cardinals = [
            d[max_idx],
            d[(max_idx + 9) % 36],
            d[(max_idx + 18) % 36],
            d[(max_idx + 27) % 36],
        ]
        max_val = max(cardinals)
        return sum(1 for v in cardinals if v < max_val) >= 3

    def _is_centered(self, d):
        max_idx = d.index(max(d))
        heading_out = (max_idx == FRONT_IDX)

        pair1, pair2 = self._find_pairs(d)
        if pair1 is None or pair2 is None:
            return False
        
        self.get_logger().info(f'max_idx={max_idx} pair1={pair1} pair2={pair2}')
        if pair1:
            self.get_logger().info(f'pair1 vals: {d[pair1[0]]:.2f} vs {d[pair1[1]]:.2f}')
        if pair2:
            self.get_logger().info(f'pair2 vals: {d[pair2[0]]:.2f} vs {d[pair2[1]]:.2f}')

        pair1_err = d[pair1[0]] - d[pair1[1]]
        pair2_err = d[pair2[0]] - d[pair2[1]]

        centered = abs(pair1_err) < CENTERED_TOLERANCE and \
                abs(pair2_err) < CENTERED_TOLERANCE

        return centered and heading_out

    def _circle_detected(self, d):
    # Starting from 270 (RIGHT_IDX), scan outward in both directions
    # looking for an abrupt jump compared to the gradual slope so far.
    # A circle opening creates a smooth decrease from 270 outward,
    # followed by a sudden drop when the arc ends.

        def find_jump(start, step):
            diffs = []
            idx = start
            for _ in range(18):  # at most half the scan
                next_idx = (idx + step) % 36
                diff = abs(d[next_idx] - d[idx])
                if diffs:
                    avg = sum(diffs) / len(diffs)
                    # Jump is abrupt if current diff >> average of previous diffs
                    #self.get_logger().info(f'idx={idx} next={next_idx} diff={diff:.2f} avg={avg:.2f} ratio={diff/avg if avg>0 else 0:.2f}')

                    if avg > 0 and diff > JUMP_FACTOR * avg:
                        return True
                diffs.append(diff)
                idx = next_idx
            return False

        # Must find a jump in both directions from 270
        jump_toward_front = find_jump(RIGHT_IDX, -1)  # 270 -> 0
        jump_toward_back = find_jump(RIGHT_IDX,  1)  # 270 -> 180
        self.get_logger().info(f'find_jump: toward_front={jump_toward_front} toward_back={jump_toward_back}')
        return jump_toward_front and jump_toward_back
    # -----------------------------------------------------------------------
    # Actions
    # -----------------------------------------------------------------------

    def _enter_circle(self, d):
        # Move toward the largest gap (the circle opening)
        max_idx = d.index(max(d))
        self._send(max_idx * 10, STEP_DISTANCE)

    def _center(self, d):
        max_idx = d.index(max(d))

        pair1, pair2 = self._find_pairs(d)
        if pair1 is None or pair2 is None:
            return

        pair1_err = d[pair1[0]] - d[pair1[1]]
        pair2_err = d[pair2[0]] - d[pair2[1]]

        if abs(pair1_err) < CENTERED_TOLERANCE and \
        abs(pair2_err) < CENTERED_TOLERANCE:
            self._send(max_idx * 10, 0.0)
            return

        if abs(pair1_err) >= abs(pair2_err):
            angle = (pair1[1] * 10) if pair1_err > 0 else (pair1[0] * 10)
            move_dist = abs(pair1_err) / 2.0
        else:
            angle = (pair2[1] * 10) if pair2_err > 0 else (pair2[0] * 10)
            move_dist = abs(pair2_err) / 2.0

        self._send(angle, move_dist)

    def _wall_follow(self, d):
        # PI controller for right-side wall following
        right_dists = d[24:31]
        wall_dist = min(right_dists)

        error = wall_dist - TARGET_WALL_DISTANCE
        self.pi_integral = max(-1.0, min(1.0, self.pi_integral + error))
        correction = 1.2 * error + 0.15 * self.pi_integral

        steer_deg = max(-60.0, min(60.0, correction * 30.0))

        if steer_deg >= 0:
            angle = int(360 - steer_deg) % 360
        else:
            angle = int(-steer_deg)

        self._send(angle, STEP_DISTANCE)

    def _wander(self, d):
        # No wall detected -> random movement
        angle = random.choice([0, 90, 180, 270])
        self._send(angle, STEP_DISTANCE)

    # -----------------------------------------------------------------------

    def _send(self, angle: float, distance: float, stop: bool = False):
        msg = Vector3()
        msg.x = float(angle)
        msg.y = float(distance)
        msg.z = 1.0 if stop else 0.0
        self.cmd_pub.publish(msg)
        self.waiting = True
        self.get_logger().info(f'cmd angle={angle:.0f} dist={distance:.2f}m stop={stop}')


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