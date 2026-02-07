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
        for other in all_circles:
            if other is circle:
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
