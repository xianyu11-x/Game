"""
Predictive scan avoidance algorithm.
Scans candidate angles and blocks those that would collide after prediction_time.
"""

import math
import avoidance_config
from avoidance_base import AvoidanceBase
from avoidance_config import AVOIDANCE_CONFIG, ANGLE_SELECTION_FUNCTIONS, normalize_angle, angular_distance


class PredictiveScanAvoidance(AvoidanceBase):
    """Predictive scan avoidance - checks candidate angles for predicted collisions."""

    def __init__(self):
        super().__init__()
        self.config = AVOIDANCE_CONFIG["predictive_scan"]
        self.name = self.config["name"]
        self.description = self.config["description"]

    def calculate_avoidance(self, circle, target_x, target_y, all_circles, delta_time):
        """Calculate avoidance by scanning candidate angles."""
        if target_x is None or target_y is None:
            if avoidance_config.DEBUG_ENABLED:
                circle["debug_allowed_angles"] = []
                circle["debug_allowed_intervals"] = []
                circle["debug_detection_radius"] = None
            return self._move_with_angle(circle, circle["angle"], delta_time)

        last_target = circle.get("reverse_last_target")
        current_target = (target_x, target_y)
        if last_target is None or last_target != current_target:
            circle["reverse_cooldown"] = 0.0
            circle["reverse_extreme_count"] = 0
            circle["concave_timer"] = 0.0
            circle["concave_last_distance"] = None
            circle["reverse_last_target"] = current_target

        target_angle = self._angle_to_target(circle, target_x, target_y)

        prediction_time = self._get_prediction_time(circle, target_x, target_y, delta_time)
        speed = self._get_scan_speed(circle)
        detection_radius = self._get_detection_radius(circle, speed, prediction_time, target_x, target_y)
        move_radius = detection_radius

        neighbors = self._get_neighbors_in_radius(circle, all_circles, detection_radius)

        minkowski_circles = self._get_minkowski_circles(circle, neighbors, prediction_time)
        blocked_intervals = self._get_blocked_intervals(circle, neighbors, speed, prediction_time)
        # Merge Minkowski-based blocked intervals from rectangular obstacles
        obstacle_blocked = self._get_obstacle_blocked_intervals(
            circle, detection_radius, speed, prediction_time)
        if obstacle_blocked:
            blocked_intervals = self._merge_intervals(blocked_intervals + obstacle_blocked)
        allowed_intervals = self._get_allowed_intervals(blocked_intervals)

        if avoidance_config.DEBUG_ENABLED:
            circle["debug_detection_radius"] = detection_radius
            circle["debug_minkowski_circles"] = minkowski_circles
            circle["debug_allowed_intervals"] = allowed_intervals
            circle["debug_allowed_angles"] = self._sample_allowed_angles(allowed_intervals)
            circle["debug_angle_step"] = self.config["angle_step_deg"]

        if not allowed_intervals:
            return self._apply_fallback(circle, target_angle, delta_time)

        selected_angle = self._select_angle_from_intervals(
            allowed_intervals,
            target_angle,
            circle["angle"],
            circle,
            minkowski_circles,
        )
        if selected_angle is None:
            return self._apply_fallback(circle, target_angle, delta_time)

        self._update_reverse_cooldown(circle, selected_angle, target_angle, delta_time)
        return self._move_with_angle(circle, selected_angle, delta_time)

    def _get_scan_speed(self, circle):
        if self.config["speed_override"] is not None:
            return self.config["speed_override"]
        return circle["speed"] * self.config["speed_factor"]

    def _get_prediction_time(self, circle, target_x, target_y, delta_time):
        base_time = self.config["prediction_time"]
        if not self.config.get("concave_detection_enabled", True):
            return base_time

        current_distance = math.hypot(target_x - circle["x"], target_y - circle["y"])
        last_distance = circle.get("concave_last_distance")
        stuck_timer = circle.get("concave_timer", 0.0)
        epsilon = self.config.get("concave_progress_epsilon", 1.0)

        if last_distance is None:
            circle["concave_last_distance"] = current_distance
            return base_time

        if current_distance < last_distance - epsilon:
            stuck_timer = max(0.0, stuck_timer - self.config.get("concave_decay", 0.5) * delta_time)
        else:
            stuck_timer += delta_time

        circle["concave_last_distance"] = current_distance
        circle["concave_timer"] = stuck_timer

        growth = self.config.get("concave_time_growth", 0.5)
        max_time = self.config.get("concave_time_max", base_time)
        return min(base_time * (1.0 + stuck_timer * growth), max_time)

    def _get_detection_radius(self, circle, speed, prediction_time, target_x, target_y):
        vt_radius = speed * prediction_time * self.config["detection_radius_factor"]
        distance_to_target = math.hypot(target_x - circle["x"], target_y - circle["y"])
        radius = min(distance_to_target, vt_radius)
        radius = max(radius, self.config["detection_radius_min"])
        radius = min(radius, self.config["detection_radius_max"])
        return radius

    def _get_neighbors_in_radius(self, circle, all_circles, radius):
        neighbors = []
        radius_sq = radius * radius
        target_ref = circle.get("target_ref")
        for other in all_circles:
            if other is circle:
                continue
            # 排除Z键标记的目标单位，避免把目标当作障碍物
            if target_ref is not None and other is target_ref:
                continue
            dx = other["x"] - circle["x"]
            dy = other["y"] - circle["y"]
            if dx * dx + dy * dy <= radius_sq:
                neighbors.append(other)
        return neighbors

    def _get_minkowski_circles(self, circle, neighbors, prediction_time):
        minkowski = []
        for other in neighbors:
            other_future = self._predict_neighbor_position(other, prediction_time)
            combined_radius = circle["radius"] + other["radius"] + self.config["collision_padding"]
            minkowski.append({
                "center": other_future,
                "radius": combined_radius,
                "source": other,
            })
        return minkowski

    def _get_blocked_intervals(self, circle, neighbors, self_speed, prediction_time):
        """Return list of blocked angle intervals (start, end) in degrees within [-180, 180]."""
        if not neighbors:
            return []

        blocked = []
        time_samples = max(4, int(self.config.get("time_samples", 16)))

        for other in neighbors:
            other_speed = self._get_neighbor_speed(other)
            other_angle = math.radians(other["angle"])
            v_other = (math.cos(other_angle) * other_speed, math.sin(other_angle) * other_speed)

            p0 = (other["x"] - circle["x"], other["y"] - circle["y"])
            base_distance = math.hypot(p0[0], p0[1])
            combined_radius = circle["radius"] + other["radius"] + self.config["collision_padding"]

            if base_distance <= 1e-6:
                return [(-180.0, 180.0)]

            for i in range(1, time_samples + 1):
                t = prediction_time * i / time_samples
                if t <= 0:
                    continue

                qx = p0[0] + v_other[0] * t
                qy = p0[1] + v_other[1] * t
                distance = math.hypot(qx, qy)
                self_distance = self_speed * t

                if self_distance <= 1e-6:
                    continue

                # No intersection at time t
                if distance > self_distance + combined_radius:
                    continue
                if distance < abs(self_distance - combined_radius):
                    if combined_radius >= self_distance + distance:
                        return [(-180.0, 180.0)]
                    continue

                cos_half = (distance ** 2 + self_distance ** 2 - combined_radius ** 2) / (2 * distance * self_distance)
                cos_half = max(-1.0, min(1.0, cos_half))
                half_angle = math.degrees(math.acos(cos_half))

                center_angle = normalize_angle(math.degrees(math.atan2(qy, qx)))
                start = normalize_angle(center_angle - half_angle)
                end = normalize_angle(center_angle + half_angle)

                if start <= end:
                    blocked.append((start, end))
                else:
                    blocked.append((start, 180.0))
                    blocked.append((-180.0, end))

        return self._merge_intervals(blocked)

    def _merge_intervals(self, intervals):
        if not intervals:
            return []
        intervals = sorted(intervals, key=lambda item: item[0])
        merged = [intervals[0]]
        for start, end in intervals[1:]:
            last_start, last_end = merged[-1]
            if start <= last_end + 1e-6:
                merged[-1] = (last_start, max(last_end, end))
            else:
                merged.append((start, end))
        return merged

    def _get_allowed_intervals(self, blocked_intervals):
        if not blocked_intervals:
            return [(-180.0, 180.0)]
        if len(blocked_intervals) == 1 and blocked_intervals[0] == (-180.0, 180.0):
            return []

        allowed = []
        current = -180.0
        for start, end in blocked_intervals:
            if start > current:
                allowed.append((current, start))
            current = max(current, end)
        if current < 180.0:
            allowed.append((current, 180.0))
        return allowed

    def _sample_allowed_angles(self, allowed_intervals):
        step = self.config["angle_step_deg"]
        sampled = []
        for start, end in allowed_intervals:
            if end < start:
                continue
            count = int((end - start) / step)
            for i in range(count + 1):
                sampled.append(normalize_angle(start + i * step))
        return sampled

    def _predict_neighbor_position(self, neighbor, prediction_time):
        neighbor_speed = self._get_neighbor_speed(neighbor)
        angle_rad = math.radians(neighbor["angle"])
        dx = math.cos(angle_rad) * neighbor_speed * prediction_time
        dy = math.sin(angle_rad) * neighbor_speed * prediction_time
        return (neighbor["x"] + dx, neighbor["y"] + dy)

    def _get_neighbor_speed(self, neighbor):
        """Return predicted neighbor speed; stationary targets contribute zero speed."""
        if neighbor.get("target_x") is None or neighbor.get("target_y") is None:
            return 0.0
        if neighbor.get("speed", 0.0) <= 0.01:
            return 0.0
        return neighbor["speed"] * self.config["neighbor_speed_factor"]

    def _is_collision(self, circle, other, future_pos, other_future):
        dx = future_pos[0] - other_future[0]
        dy = future_pos[1] - other_future[1]
        distance_sq = dx * dx + dy * dy
        min_dist = circle["radius"] + other["radius"] + self.config["collision_padding"]
        return distance_sq <= min_dist * min_dist

    def _select_angle_from_intervals(self, allowed_intervals, target_angle, current_angle, circle, minkowski_circles):
        method = self.config["angle_selection_method"]
        selector = ANGLE_SELECTION_FUNCTIONS.get(method)
        if selector is None:
            return None

        candidates = []
        for start, end in allowed_intervals:
            candidates.append(start)
            candidates.append(end)
            if start <= target_angle <= end:
                candidates.append(target_angle)
            if start <= current_angle <= end:
                candidates.append(current_angle)

        if not candidates:
            return None

        penalty_weight = self.config.get("turn_penalty_weight", 0.0)
        threshold = self.config.get("turn_penalty_threshold_deg", 0.0)
        if penalty_weight < 0.0:
            penalty_weight = 0.0

        def penalty(angle):
            diff = angular_distance(angle, current_angle)
            if diff <= threshold:
                return 0.0
            return (diff - threshold) * penalty_weight

        def score(angle):
            return angular_distance(angle, target_angle) + penalty(angle)

        best_angle = min(candidates, key=score)

        if not self.config.get("opposite_suppression_enabled", True):
            return best_angle

        opposite_threshold = self.config.get("opposite_angle_threshold_deg", 30.0)
        opposite_margin = self.config.get("opposite_preference_margin", 15.0)
        opposite_dir = normalize_angle(current_angle + 180.0)

        if self.config.get("reverse_cooldown_enabled", True):
            cooldown = circle.get("reverse_cooldown", 0.0)
            if cooldown > 0.0:
                safe_candidates = [
                    a for a in candidates
                    if angular_distance(a, opposite_dir) > opposite_threshold
                ]
                if safe_candidates:
                    best_angle = min(safe_candidates, key=score)

        non_opposite = [a for a in candidates if angular_distance(a, opposite_dir) > opposite_threshold]
        if not non_opposite:
            return best_angle

        best_non_opposite = min(non_opposite, key=score)
        if score(best_angle) + opposite_margin < score(best_non_opposite):
            return best_angle
        return best_non_opposite

    def _update_reverse_cooldown(self, circle, selected_angle, target_angle, delta_time):
        if not self.config.get("reverse_cooldown_enabled", True):
            return

        base = self.config.get("reverse_cooldown_base", 0.4)
        growth = self.config.get("reverse_cooldown_growth", 2.0)
        decay = self.config.get("reverse_cooldown_decay", 0.6)
        max_cooldown = self.config.get("reverse_cooldown_max", 3.0)
        extreme_threshold = self.config.get("reverse_extreme_threshold_deg", 120.0)

        cooldown = circle.get("reverse_cooldown", 0.0)
        extreme_count = circle.get("reverse_extreme_count", 0)

        if angular_distance(selected_angle, target_angle) >= extreme_threshold:
            extreme_count += 1
            cooldown = max(cooldown, min(base * (growth ** (extreme_count - 1)), max_cooldown))
        else:
            extreme_count = max(0, extreme_count - 1)
            cooldown = max(0.0, cooldown - decay * delta_time)

        circle["reverse_cooldown"] = cooldown
        circle["reverse_extreme_count"] = extreme_count


    @staticmethod
    def _clip_segment_to_circle(x1, y1, x2, y2, cx, cy, r):
        """Return the portion of segment (x1,y1)→(x2,y2) inside circle (cx,cy,r).
        
        Returns (px1, py1, px2, py2) or None if no overlap.
        """
        dx, dy = x2 - x1, y2 - y1
        fx, fy = x1 - cx, y1 - cy
        a = dx * dx + dy * dy
        if a < 1e-12:
            if fx * fx + fy * fy <= r * r:
                return (x1, y1, x2, y2)
            return None
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - r * r
        disc = b * b - 4.0 * a * c
        if disc < 0:
            return None
        sd = math.sqrt(disc)
        t1 = (-b - sd) / (2.0 * a)
        t2 = (-b + sd) / (2.0 * a)
        t_lo = max(0.0, t1)
        t_hi = min(1.0, t2)
        if t_lo > t_hi + 1e-9:
            return None
        return (x1 + t_lo * dx, y1 + t_lo * dy,
                x1 + t_hi * dx, y1 + t_hi * dy)

    def _get_obstacle_blocked_intervals(self, circle, detection_radius,
                                        self_speed, prediction_time):
        """Compute blocked angle intervals from rectangular obstacles.
        
        Decomposes each obstacle's Minkowski sum (rounded rectangle) into
        4 straight edge segments + 4 corner circles.
        
        Edge segments: clipped to detection circle, angles to clipped endpoints.
        Corner circles: same time-parametric intersection logic as circle-circle
                        detection (_get_blocked_intervals), ensuring consistent
                        behaviour and natural detection-radius clipping.
        """
        if not self.obstacles:
            return []
        
        blocked = []
        ax, ay = circle["x"], circle["y"]
        expand = circle["radius"] + self.config["collision_padding"]
        det_r = detection_radius
        
        for obs in self.obstacles:
            hw, hh = obs["w"] / 2.0, obs["h"] / 2.0
            
            # Transform agent into obstacle local space
            neg_rad = math.radians(-obs["angle"])
            cos_n = math.cos(neg_rad)
            sin_n = math.sin(neg_rad)
            ddx = ax - obs["cx"]
            ddy = ay - obs["cy"]
            lx = ddx * cos_n - ddy * sin_n
            ly = ddx * sin_n + ddy * cos_n
            
            # Inside rounded rectangle check
            inside = False
            if abs(lx) <= hw and abs(ly) <= hh + expand:
                inside = True
            elif abs(lx) <= hw + expand and abs(ly) <= hh:
                inside = True
            else:
                cdx = abs(lx) - hw
                cdy = abs(ly) - hh
                if cdx > 0 and cdy > 0 and cdx * cdx + cdy * cdy <= expand * expand:
                    inside = True
            
            if inside:
                # Block towards-obstacle half, allow escape outward
                esc = math.degrees(math.atan2(ddy, ddx))
                esc = ((esc + 180.0) % 360.0) - 180.0
                bs = ((esc + 90.0 + 180.0) % 360.0) - 180.0
                be = ((esc - 90.0 + 180.0) % 360.0) - 180.0
                if bs <= be:
                    blocked.append((bs, be))
                else:
                    blocked.append((bs, 180.0))
                    blocked.append((-180.0, be))
                continue
            
            # Quick cull: closest point on original rect surface
            cp_x = max(-hw, min(hw, lx))
            cp_y = max(-hh, min(hh, ly))
            if math.hypot(lx - cp_x, ly - cp_y) > det_r + expand:
                continue
            
            # World-space transform
            obs_rad = math.radians(obs["angle"])
            cos_o = math.cos(obs_rad)
            sin_o = math.sin(obs_rad)
            
            def tw(plx, ply):
                return (obs["cx"] + plx * cos_o - ply * sin_o,
                        obs["cy"] + plx * sin_o + ply * cos_o)
            
            e = expand
            angles = []
            
            # --- 4 edge segments, clipped to detection circle ---
            segs = [
                ((-hw, -hh - e), ( hw, -hh - e)),   # bottom
                (( hw + e, -hh), ( hw + e,  hh)),    # right
                (( hw,  hh + e), (-hw,  hh + e)),    # top
                ((-hw - e,  hh), (-hw - e, -hh)),    # left
            ]
            for (sl1, sl2) in segs:
                w1 = tw(*sl1)
                w2 = tw(*sl2)
                clipped = self._clip_segment_to_circle(
                    w1[0], w1[1], w2[0], w2[1], ax, ay, det_r)
                if clipped is None:
                    continue
                a1 = math.degrees(math.atan2(clipped[1] - ay, clipped[0] - ax))
                a2 = math.degrees(math.atan2(clipped[3] - ay, clipped[2] - ax))
                angles.append(((a1 + 180.0) % 360.0) - 180.0)
                angles.append(((a2 + 180.0) % 360.0) - 180.0)
            
            # --- 4 corner circles (static obstacle: no time sampling needed) ---
            # Corner circle: static, center at rect corner, radius = expand.
            # Since the obstacle doesn't move, the relative position is constant
            # across all prediction times. We only need to compute once using
            # self_distance = self_speed * prediction_time (maximum reach).
            corners = [(hw, hh), (-hw, hh), (-hw, -hh), (hw, -hh)]
            self_distance = self_speed * prediction_time
            for clx, cly in corners:
                wc = tw(clx, cly)
                p0x = wc[0] - ax
                p0y = wc[1] - ay
                base_dist = math.hypot(p0x, p0y)
                
                if base_dist <= 1e-6:
                    return [(-180.0, 180.0)]
                if base_dist <= e:
                    return [(-180.0, 180.0)]
                if base_dist - e > det_r:
                    continue  # corner circle fully outside detection
                
                if self_distance <= 1e-6:
                    continue
                # Static obstacle: distance is constant = base_dist
                if base_dist > self_distance + e:
                    continue
                if base_dist < abs(self_distance - e):
                    if e >= self_distance + base_dist:
                        return [(-180.0, 180.0)]
                    continue
                
                cos_half = (base_dist * base_dist + self_distance * self_distance - e * e) / (2.0 * base_dist * self_distance)
                cos_half = max(-1.0, min(1.0, cos_half))
                half_angle = math.degrees(math.acos(cos_half))
                
                center_angle = normalize_angle(math.degrees(math.atan2(p0y, p0x)))
                start = normalize_angle(center_angle - half_angle)
                end = normalize_angle(center_angle + half_angle)
                
                if start <= end:
                    blocked.append((start, end))
                else:
                    blocked.append((start, 180.0))
                    blocked.append((-180.0, end))
            
            # Edge segment angles → largest-gap → blocked interval
            if len(angles) >= 2:
                angles.sort()
                n = len(angles)
                max_gap = -1.0
                max_gap_idx = 0
                for i in range(n):
                    if i < n - 1:
                        gap = angles[i + 1] - angles[i]
                    else:
                        gap = (angles[0] + 360.0) - angles[n - 1]
                    if gap > max_gap:
                        max_gap = gap
                        max_gap_idx = i
                
                b_start = angles[(max_gap_idx + 1) % n]
                b_end = angles[max_gap_idx]
                if b_start <= b_end:
                    blocked.append((b_start, b_end))
                else:
                    blocked.append((b_start, 180.0))
                    blocked.append((-180.0, b_end))
        
        return self._merge_intervals(blocked) if blocked else []

    def _apply_fallback(self, circle, target_angle, delta_time):
        fallback = self.config["fallback_method"]
        if fallback == "stop":
            return (circle["x"], circle["y"], circle["angle"])
        if fallback == "target":
            return self._move_with_angle(circle, target_angle, delta_time)
        return self._move_with_angle(circle, circle["angle"], delta_time)

    def _move_with_angle(self, circle, angle, delta_time):
        move_distance = circle["speed"] * delta_time
        angle_rad = math.radians(angle)
        new_x = circle["x"] + move_distance * math.cos(angle_rad)
        new_y = circle["y"] + move_distance * math.sin(angle_rad)
        return (new_x, new_y, angle)


    def _angle_to_target(self, circle, target_x, target_y):
        dx = target_x - circle["x"]
        dy = target_y - circle["y"]
        return normalize_angle(math.degrees(math.atan2(dy, dx)))
