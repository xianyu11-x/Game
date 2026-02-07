"""
Predictive scan avoidance algorithm with parallelized blocked interval scanning.
"""

from __future__ import annotations

import math
import os
from concurrent.futures import ThreadPoolExecutor

from avoidance_config import AVOIDANCE_CONFIG
from avoidance_predictive import PredictiveScanAvoidance


class PredictiveScanAvoidanceParallel(PredictiveScanAvoidance):
    """Predictive scan avoidance with parallel blocked interval evaluation."""

    def __init__(self):
        super().__init__()
        self.config = AVOIDANCE_CONFIG["predictive_scan_parallel"]
        self.name = self.config["name"]
        self.description = self.config["description"]

    def _get_blocked_intervals(self, circle, neighbors, self_speed, prediction_time):
        if not neighbors:
            return []

        time_samples = max(4, int(self.config.get("time_samples", 16)))
        max_workers = int(self.config.get("parallel_workers", 0))
        if max_workers <= 0:
            max_workers = min(32, (os.cpu_count() or 1))

        if max_workers <= 1 or len(neighbors) < 2:
            return super()._get_blocked_intervals(circle, neighbors, self_speed, prediction_time)

        def evaluate_neighbor(other):
            other_speed = self._get_neighbor_speed(other)
            other_angle = math.radians(other["angle"])
            v_other = (math.cos(other_angle) * other_speed, math.sin(other_angle) * other_speed)

            p0 = (other["x"] - circle["x"], other["y"] - circle["y"])
            base_distance = math.hypot(p0[0], p0[1])
            combined_radius = circle["radius"] + other["radius"] + self.config["collision_padding"]

            if base_distance <= 1e-6:
                return True, [(-180.0, 180.0)]

            blocked = []
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

                if distance > self_distance + combined_radius:
                    continue
                if distance < abs(self_distance - combined_radius):
                    if combined_radius >= self_distance + distance:
                        return True, [(-180.0, 180.0)]
                    continue

                cos_half = (distance ** 2 + self_distance ** 2 - combined_radius ** 2) / (
                    2 * distance * self_distance
                )
                cos_half = max(-1.0, min(1.0, cos_half))
                half_angle = math.degrees(math.acos(cos_half))

                center_angle = self._normalize_center_angle(qx, qy)
                start = self._normalize_center_angle_delta(center_angle - half_angle)
                end = self._normalize_center_angle_delta(center_angle + half_angle)

                if start <= end:
                    blocked.append((start, end))
                else:
                    blocked.append((start, 180.0))
                    blocked.append((-180.0, end))

            return False, blocked

        blocked_intervals = []
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            results = executor.map(evaluate_neighbor, neighbors)
            for is_full, intervals in results:
                if is_full:
                    return [(-180.0, 180.0)]
                blocked_intervals.extend(intervals)

        return self._merge_intervals(blocked_intervals)

    @staticmethod
    def _normalize_center_angle(qx, qy):
        from avoidance_config import normalize_angle

        return normalize_angle(math.degrees(math.atan2(qy, qx)))

    @staticmethod
    def _normalize_center_angle_delta(angle):
        from avoidance_config import normalize_angle

        return normalize_angle(angle)
