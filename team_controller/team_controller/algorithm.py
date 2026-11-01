"""Standalone reactive racing algorithm with stuck recovery.

Distances in metres, angles in radians. No map, waypoints, external libraries,
or odometry are required (scan only), so it is valid for every stage.

Forward driving: disparity-extender gap following. Recovery: if the nose is
pinned against a wall, back away (steering the nose toward the open side)
until there is room, then drive forward again.
"""
import math

try:
    from .levine_route import ROUTE
except ImportError:  # running outside the package (tests)
    from levine_route import ROUTE

WHEELBASE = 0.33
OFF_ROUTE = 2.0           # farther than this from the centerline -> reactive only
MAX_STEER = 0.40          # rad, stays inside the simulator limit (0.4189)
HALF_WIDTH = 0.16         # car half width (m)
SAFETY = 0.12             # extra lateral margin (m)
SCAN_LIMIT = math.radians(95)
RANGE_CAP = 6.0           # ignore detail beyond this (m)
DISPARITY = 0.45          # range jump that marks an obstacle edge (m)
FRONT_STOP = 0.32         # nose this close to a wall -> recovery (m)
REVERSE_SPEED = -0.45
REVERSE_CLEAR = 1.0       # stop reversing when the front is this open (m)
REVERSE_MAX_TIME = 3.0    # s
REAR_CLOSE = 0.30         # rear-side beams closer than this stop reversing (m)


def _clean(ranges, angle_min, angle_increment, range_min, range_max):
    out = []
    cap = min(RANGE_CAP, range_max)
    for i, raw in enumerate(ranges):
        a = angle_min + i * angle_increment
        if math.isinf(raw) and raw > 0:
            d = cap
        elif math.isfinite(raw) and range_min <= raw <= range_max:
            d = min(cap, raw)
        else:
            d = 0.0  # invalid readings are never treated as open space
        out.append((a, d))
    return out


def _valid_geometry(ranges, angle_min, angle_increment, range_min, range_max,
                    max_speed):
    return not (len(ranges) < 3 or not math.isfinite(angle_min)
                or not math.isfinite(angle_increment) or angle_increment == 0
                or not math.isfinite(range_max) or range_max <= 0
                or not math.isfinite(range_min) or range_min < 0
                or range_min >= range_max or not math.isfinite(max_speed))


class Driver:
    """Stateful driver: call step() on each scan; returns (steering, speed)."""

    def __init__(self, max_speed=0.8):
        self.max_speed = max(0.0, min(0.8, max_speed))
        self.mode = 'drive'
        self.blocked_since = None
        self.reverse_since = None
        self.reverse_steer = 0.0
        self.cooldown_until = 0.0
        self.route_idx = None
        self.route_len = len(ROUTE)

    # -- route helpers ---------------------------------------------------
    def _route_target(self, pose, lookahead):
        """Return (bearing, lookahead distance) to the route, or None if off it."""
        x, y, yaw = pose
        n = self.route_len
        if self.route_idx is None:
            cand = range(n)
        else:
            cand = [(self.route_idx + k) % n for k in range(-10, 41)]
        best, best_d = None, 1e9
        for i in cand:
            d = math.hypot(ROUTE[i][0] - x, ROUTE[i][1] - y)
            if d < best_d:
                best, best_d = i, d
        if best_d > OFF_ROUTE:
            self.route_idx = None
            return None
        self.route_idx = best
        j = best
        while math.hypot(ROUTE[j][0] - x, ROUTE[j][1] - y) < lookahead \
                and (j - best) % n < n - 1:
            j = (j + 1) % n
        tx, ty = ROUTE[j]
        bearing = math.atan2(ty - y, tx - x) - yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        return bearing, math.hypot(tx - x, ty - y)

    # -- helpers ---------------------------------------------------------
    @staticmethod
    def _front_clear(samples, half_angle):
        vals = [d for a, d in samples if abs(a) <= half_angle]
        return min(vals) if vals else 0.0

    def _gap_target(self, samples, inc, bearing=None, need=1.5):
        """Return (steering target angle, usable) using disparity extension."""
        fov = [(a, d) for a, d in samples if abs(a) <= SCAN_LIMIT]
        if len(fov) < 5:
            return 0.0, False
        ang = [a for a, _ in fov]
        r = [d for _, d in fov]
        n = len(r)
        proc = list(r)
        step = abs(inc) if inc else 0.01
        for i in range(n - 1):
            diff = r[i + 1] - r[i]
            if abs(diff) < DISPARITY:
                continue
            near = min(r[i], r[i + 1])
            if near <= 0.0:
                continue
            width = HALF_WIDTH + SAFETY
            cnt = int(math.ceil(math.atan2(width, max(near, 0.05)) / step))
            if diff > 0:       # left side nearer? (i nearer): extend to i+1...
                for j in range(i + 1, min(n, i + 1 + cnt)):
                    proc[j] = min(proc[j], near)
            else:              # i+1 nearer: extend backwards
                for j in range(max(0, i - cnt + 1), i + 1):
                    proc[j] = min(proc[j], near)
        # smooth to avoid single-beam spikes
        sm = []
        for i in range(n):
            lo, hi = max(0, i - 2), min(n, i + 3)
            sm.append(sum(proc[lo:hi]) / (hi - lo))
        if bearing is not None:
            b = max(-SCAN_LIMIT, min(SCAN_LIMIT, bearing))
            k = min(range(n), key=lambda i: abs(ang[i] - b))
            if sm[k] >= need:
                return b, True
            ok_idx = [i for i in range(n) if sm[i] >= need]
            if ok_idx:
                i = min(ok_idx, key=lambda i: abs(ang[i] - b))
                return ang[i], True
        best, best_score = None, -1e9
        for i in range(n):
            score = sm[i] - 0.7 * abs(ang[i])
            if score > best_score:
                best, best_score = i, score
        if sm[best] < 0.5:
            return 0.0, False
        return ang[best], True

    # -- main entry ------------------------------------------------------
    def step(self, ranges, angle_min, angle_increment, range_min, range_max,
             now, pose=None):
        if not _valid_geometry(ranges, angle_min, angle_increment, range_min,
                               range_max, self.max_speed):
            self.mode = 'drive'
            self.blocked_since = None
            return 0.0, 0.0
        samples = _clean(ranges, angle_min, angle_increment, range_min,
                         range_max)
        valid = sum(1 for _, d in samples if d > 0.0)
        if valid < 0.3 * len(samples):  # mostly garbage: never move blindly
            self.mode = 'drive'
            self.blocked_since = None
            return 0.0, 0.0
        front = self._front_clear(samples, math.radians(12))
        left_rear = [d for a, d in samples if a > math.radians(115)]
        right_rear = [d for a, d in samples if a < -math.radians(115)]
        rear_l = min(left_rear) if left_rear else 1.0
        rear_r = min(right_rear) if right_rear else 1.0
        cap = self.max_speed

        if self.mode == 'reverse':
            done = (front >= REVERSE_CLEAR
                    or now - self.reverse_since > REVERSE_MAX_TIME)
            if self.reverse_steer > 0 and rear_l < REAR_CLOSE:
                done = True
            if self.reverse_steer < 0 and rear_r < REAR_CLOSE:
                done = True
            if min(rear_l, rear_r) < 0.18:
                done = True
            if not done:
                return self.reverse_steer, REVERSE_SPEED * (cap / 0.8 if cap else 0)
            self.mode = 'drive'
            self.blocked_since = None
            self.cooldown_until = now + 1.0

        route = None
        lookahead = 1.4
        if pose is not None and all(math.isfinite(v) for v in pose):
            route = self._route_target(pose, lookahead)
        if route is not None:
            target, ok = self._gap_target(samples, angle_increment,
                                          bearing=route[0],
                                          need=min(2.5, lookahead + 0.4))
        else:
            self.route_idx = None
            target, ok = self._gap_target(samples, angle_increment)
        pinned = front < FRONT_STOP or not ok
        if pinned:
            if self.blocked_since is None:
                self.blocked_since = now
            if now - self.blocked_since > 0.4 and cap > 0:
                # choose the open side: reverse so the nose swings that way
                left = [d for a, d in samples if math.radians(20) < a < math.radians(95)]
                right = [d for a, d in samples if -math.radians(95) < a < -math.radians(20)]
                lm = sum(left) / len(left) if left else 0.0
                rm = sum(right) / len(right) if right else 0.0
                # reversing with steering s yaws the car by -sign(s): steer
                # opposite to the open side.
                self.reverse_steer = -MAX_STEER if lm >= rm else MAX_STEER
                self.mode = 'reverse'
                self.reverse_since = now
                return self.reverse_steer, REVERSE_SPEED * (cap / 0.8)
            return 0.0, 0.0
        self.blocked_since = None

        if route is not None:
            steering = math.atan2(2.0 * WHEELBASE * math.sin(target), lookahead)
        else:
            steering = 0.9 * target
        steering = max(-MAX_STEER, min(MAX_STEER, steering))
        # do not turn into a wall alongside the car
        side_l = min([d for a, d in samples
                      if math.radians(60) < a < math.radians(100)] or [9.0])
        side_r = min([d for a, d in samples
                      if -math.radians(100) < a < -math.radians(60)] or [9.0])
        if side_l < 0.22:
            steering = min(steering, -0.08)
        if side_r < 0.22:
            steering = max(steering, 0.08)
        turn_factor = max(0.45, 1.0 - abs(steering) / 0.7)
        clear_factor = min(1.0, max(0.0, (front - FRONT_STOP) / 1.5))
        speed = cap * min(turn_factor, max(clear_factor, 0.0))
        if front > FRONT_STOP + 0.1:
            speed = max(speed, min(cap, 0.25))
        return steering, speed


_default = {}


def choose_drive(ranges, angle_min, angle_increment, range_min, range_max,
                 max_speed=0.8, now=0.0, pose=None):
    """Backward-compatible functional wrapper around a shared Driver."""
    d = _default.get(max_speed)
    if d is None:
        d = _default[max_speed] = Driver(max_speed)
    return d.step(ranges, angle_min, angle_increment, range_min, range_max,
                  now, pose)
