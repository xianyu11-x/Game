"""
Minkowski Sum Tangent-based avoidance algorithm.

Flow:
1. Compute Minkowski sum circles for each neighbor (radius = self.r + other.r)
   and each rectangular obstacle (bounding-circle approximation).
2. Test whether the straight line from self to target is blocked by any
   Minkowski circle.
3. If blocked, group overlapping Minkowski circles via Union-Find
   (connected components where two circles intersect).
4. Compute blocked angular intervals from all nearby Minkowski circles,
   derive the allowed (passable) gaps, and pick the tangent angle closest
   to the target direction.
5. Move in that direction.
"""

import math
import avoidance_config
from avoidance_base import AvoidanceBase
from avoidance_config import AVOIDANCE_CONFIG, normalize_angle, angular_distance


class MinkowskiTangentAvoidance(AvoidanceBase):
    """Tangent-based avoidance using Minkowski sum obstacle grouping."""

    def __init__(self):
        super().__init__()
        self.config = AVOIDANCE_CONFIG["minkowski_tangent"]
        self.name = self.config["name"]
        self.description = self.config["description"]

    # ------------------------------------------------------------------
    #  Public interface
    # ------------------------------------------------------------------

    def calculate_avoidance(self, circle, target_x, target_y, all_circles, delta_time):
        if target_x is None or target_y is None:
            self._clear_debug(circle)
            return self._move_with_angle(circle, circle["angle"], delta_time)

        ax, ay = circle["x"], circle["y"]
        target_angle = normalize_angle(
            math.degrees(math.atan2(target_y - ay, target_x - ax))
        )
        det_r = self.config["detection_radius"]

        # Step 1 – build Minkowski-sum circles
        mink_circles = self._build_minkowski_circles(circle, all_circles, det_r)

        if not mink_circles:
            self._clear_debug(circle)
            return self._move_with_angle(circle, target_angle, delta_time)

        # Step 2 – line-of-sight check
        if not self._segment_blocked(ax, ay, target_x, target_y, mink_circles):
            self._set_debug(circle, mink_circles, [], [], [],
                            [(-180.0, 180.0)], det_r)
            return self._move_with_angle(circle, target_angle, delta_time)

        # Step 3 – group overlapping Minkowski circles (Union-Find)
        groups = self._union_find_groups(mink_circles)

        blocking_flags = [
            self._segment_blocked(ax, ay, target_x, target_y, g)
            for g in groups
        ]

        # Step 4 – blocked angular intervals only from groups hit by the line
        margin = self.config.get("tangent_margin_deg", 3.0)
        raw_blocked = []
        for idx, group in enumerate(groups):
            if not blocking_flags[idx]:
                continue
            for mc in group:
                raw_blocked.extend(self._angular_block(ax, ay, mc, margin))

        blocked = self._merge_intervals(raw_blocked)
        allowed = self._allowed_intervals(blocked)

        self._set_debug(circle, mink_circles, groups, blocking_flags,
                        blocked, allowed, det_r)

        if not allowed:
            return self._fallback(circle, delta_time)

        best = self._pick_tangent(allowed, target_angle, circle["angle"])
        if best is None:
            return self._fallback(circle, delta_time)

        return self._move_with_angle(circle, best, delta_time)

    # ------------------------------------------------------------------
    #  Step 1 – Minkowski sum construction
    # ------------------------------------------------------------------

    def _build_minkowski_circles(self, circle, all_circles, det_r):
        ax, ay = circle["x"], circle["y"]
        pad = self.config.get("collision_padding", 0.0)
        target_ref = circle.get("target_ref")
        result = []

        for other in all_circles:
            if other is circle:
                continue
            if target_ref is not None and other is target_ref:
                continue
            dx, dy = other["x"] - ax, other["y"] - ay
            dist = math.hypot(dx, dy)
            r = circle["radius"] + other["radius"] + pad
            if dist > det_r + r:
                continue
            result.append({"cx": other["x"], "cy": other["y"], "radius": r})

        for obs in self.obstacles:
            dx, dy = obs["cx"] - ax, obs["cy"] - ay
            dist = math.hypot(dx, dy)
            half_diag = math.hypot(obs["w"] / 2.0, obs["h"] / 2.0)
            r = circle["radius"] + half_diag + pad
            if dist > det_r + r:
                continue
            result.append({"cx": obs["cx"], "cy": obs["cy"], "radius": r})

        return result

    # ------------------------------------------------------------------
    #  Step 2 – segment / circle intersection
    # ------------------------------------------------------------------

    @staticmethod
    def _segment_blocked(x1, y1, x2, y2, circles):
        for c in circles:
            if MinkowskiTangentAvoidance._seg_circle_hit(
                    x1, y1, x2, y2, c["cx"], c["cy"], c["radius"]):
                return True
        return False

    @staticmethod
    def _seg_circle_hit(x1, y1, x2, y2, cx, cy, r):
        dx, dy = x2 - x1, y2 - y1
        fx, fy = x1 - cx, y1 - cy
        a = dx * dx + dy * dy
        if a < 1e-12:
            return fx * fx + fy * fy <= r * r
        b = 2.0 * (fx * dx + fy * dy)
        c_val = fx * fx + fy * fy - r * r
        if c_val <= 0:
            return True
        disc = b * b - 4.0 * a * c_val
        if disc < 0:
            return False
        sd = math.sqrt(disc)
        t1 = (-b - sd) / (2.0 * a)
        t2 = (-b + sd) / (2.0 * a)
        return t1 <= 1.0 and t2 >= 0.0

    # ------------------------------------------------------------------
    #  Step 3 – Union-Find grouping
    # ------------------------------------------------------------------

    @staticmethod
    def _union_find_groups(circles):
        n = len(circles)
        if n == 0:
            return []
        parent = list(range(n))
        rnk = [0] * n

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def union(a, b):
            ra, rb = find(a), find(b)
            if ra == rb:
                return
            if rnk[ra] < rnk[rb]:
                ra, rb = rb, ra
            parent[rb] = ra
            if rnk[ra] == rnk[rb]:
                rnk[ra] += 1

        for i in range(n):
            ci = circles[i]
            for j in range(i + 1, n):
                cj = circles[j]
                dist = math.hypot(ci["cx"] - cj["cx"], ci["cy"] - cj["cy"])
                if dist < ci["radius"] + cj["radius"]:
                    union(i, j)

        groups_map = {}
        for i in range(n):
            groups_map.setdefault(find(i), []).append(circles[i])
        return list(groups_map.values())

    # ------------------------------------------------------------------
    #  Step 4 – angular blocking & interval math
    # ------------------------------------------------------------------

    @staticmethod
    def _angular_block(ax, ay, mc, margin_deg=0.0):
        """Return blocked angular interval(s) of *mc* as seen from (ax, ay)."""
        dx = mc["cx"] - ax
        dy = mc["cy"] - ay
        dist = math.hypot(dx, dy)

        if dist < 1e-6:
            return [(-180.0, 180.0)]

        if dist <= mc["radius"]:
            toward = normalize_angle(math.degrees(math.atan2(dy, dx)))
            s = normalize_angle(toward - 90.0)
            e = normalize_angle(toward + 90.0)
            if s <= e:
                return [(s, e)]
            return [(s, 180.0), (-180.0, e)]

        half = math.degrees(math.asin(min(1.0, mc["radius"] / dist))) + margin_deg
        center = normalize_angle(math.degrees(math.atan2(dy, dx)))
        s = normalize_angle(center - half)
        e = normalize_angle(center + half)
        if s <= e:
            return [(s, e)]
        return [(s, 180.0), (-180.0, e)]

    @staticmethod
    def _merge_intervals(intervals):
        if not intervals:
            return []
        intervals = sorted(intervals, key=lambda i: i[0])
        merged = [intervals[0]]
        for s, e in intervals[1:]:
            ls, le = merged[-1]
            if s <= le + 1e-6:
                merged[-1] = (ls, max(le, e))
            else:
                merged.append((s, e))
        return merged

    @staticmethod
    def _allowed_intervals(blocked):
        if not blocked:
            return [(-180.0, 180.0)]
        full_block = (len(blocked) == 1
                      and blocked[0][0] <= -179.99
                      and blocked[0][1] >= 179.99)
        if full_block:
            return []
        allowed = []
        cur = -180.0
        for s, e in blocked:
            if s > cur + 0.01:
                allowed.append((cur, s))
            cur = max(cur, e)
        if cur < 180.0 - 0.01:
            allowed.append((cur, 180.0))
        return allowed

    # ------------------------------------------------------------------
    #  Angle selection
    # ------------------------------------------------------------------

    @staticmethod
    def _pick_tangent(allowed, target_angle, current_angle):
        if not allowed:
            return None
        candidates = []
        for s, e in allowed:
            candidates.append(s)
            candidates.append(e)
            if s <= target_angle <= e:
                candidates.append(target_angle)
            if s <= current_angle <= e:
                candidates.append(current_angle)
        if not candidates:
            return None
        return min(candidates, key=lambda a: angular_distance(a, target_angle))

    # ------------------------------------------------------------------
    #  Movement helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _move_with_angle(circle, angle, dt):
        dist = circle["speed"] * dt
        rad = math.radians(angle)
        return (circle["x"] + dist * math.cos(rad),
                circle["y"] + dist * math.sin(rad),
                angle)

    @staticmethod
    def _fallback(circle, dt):
        return (circle["x"], circle["y"], circle["angle"])

    # ------------------------------------------------------------------
    #  Debug helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _clear_debug(circle):
        for key in ("debug_detection_radius", "debug_minkowski_circles",
                     "debug_allowed_intervals", "debug_allowed_angles",
                     "debug_angle_step", "debug_mink_groups"):
            circle.pop(key, None)

    @staticmethod
    def _set_debug(circle, mink_circles, groups, blocking_flags,
                   blocked, allowed, det_r):
        if not avoidance_config.DEBUG_ENABLED:
            MinkowskiTangentAvoidance._clear_debug(circle)
            return

        circle["debug_detection_radius"] = det_r
        circle["debug_minkowski_circles"] = [
            {"center": (mc["cx"], mc["cy"]), "radius": mc["radius"]}
            for mc in mink_circles
        ]
        circle["debug_allowed_intervals"] = allowed
        circle["debug_allowed_angles"] = []
        circle["debug_angle_step"] = 5.0

        group_info = []
        for idx, g in enumerate(groups):
            is_blocking = blocking_flags[idx] if idx < len(blocking_flags) else False
            group_info.append({
                "circles": [
                    {"center": (mc["cx"], mc["cy"]), "radius": mc["radius"]}
                    for mc in g
                ],
                "blocking": is_blocking,
            })
        circle["debug_mink_groups"] = group_info
